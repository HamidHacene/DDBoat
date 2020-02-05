#include <cmath>
#include "ros/ros.h"
#include "tf/tf.h"
#include "filters/GPose.h"
#include "gpsd_client/GnssPose.h"
#include "geometry_msgs/Vector3.h"
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

Vector2d ykal;
Vector4d ukal;

void kalman_predict(Vector4d& x1, Matrix4d& Gx1, Vector4d& xup, Matrix4d& Gup, Vector4d& u, Matrix4d& Galpha, Matrix4d& A)
{
    Gx1 = A * Gup * A.transpose() + Galpha;
    x1 = A * xup + u;
}

void kalman_correct(Vector4d&xup, Matrix4d& Gup, Vector4d& x0, Matrix4d& Gx0, Vector2d& y, Matrix2d& Gbeta, MatrixXd& C)
{
    MatrixXd S = C * Gx0 * C.transpose() + Gbeta;
    MatrixXd K = Gx0 * C.transpose() * S.inverse();
    VectorXd ytilde = y - C * x0;
    Gup = (MatrixXd::Identity(x0.size(), x0.size()) - K * C) * Gx0;
    xup = x0 + K * ytilde;
}

void kalman(Vector4d& x0, Matrix4d& Gx0, Vector4d& u, Matrix4d& Galpha, Matrix4d& A, Vector2d& y, Matrix2d& Gbeta, MatrixXd& C){
    Vector4d xup;
    Matrix4d Gup;
    kalman_correct(xup, Gup, x0, Gx0, y, Gbeta, C);
    kalman_predict(x0, Gx0, xup, Gup, u, Galpha, A);
}

void commandCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ukal << 0, 0, msg->x, msg->y;
}

void lambertCallback(const gpsd_client::GnssPose::ConstPtr& msg)
{
    ykal << msg->east, msg->north;
}

int main(int argc, char **argv)
{
    //Initialisation Kalman
    // ------------------------------------------
    Vector4d x0 = {253571.951544, 6805722.31735, 0., 0.1}; // doit être initialisé correctement dans le launch !
    Matrix4d Gx0 = 10 * MatrixXd::Identity(4, 4);
    Matrix4d Galpha = MatrixXd::Zero(4, 4);
    MatrixXd C(2, 4);
    C << 1., 0., 0., 0., 0., 1., 0., 0.;
    Matrix2d B;
    B << 1, -1, 1, 1;
    Matrix2d Gbeta,theta1;
    Gbeta << 5, 0, 0, 5;
    // ------------------------------------------

    const double dt = 0.1;
    ros::init(argc, argv, "kalman");
    ros::NodeHandle n;
    n.param<double>("pos_x", x0(0), 253571.951544);
    n.param<double>("pos_y", x0(1), 6805722.31735);
    n.param<double>("yaw", x0(2), 0.0);
    n.param<double>("yaw", x0(3), 0.1);

    //Abonnement aux topics
    ros::Publisher estimated_state_pub = n.advertise<filters::GPose>("poseCorrected", 1000);
    ros::Subscriber com_sub = n.subscribe("command", 1000, commandCallback); // message de type Vector3
    ros::Subscriber lamb_sub = n.subscribe("poseRaw", 1000, lambertCallback); // message de type Vector 3
    ros::Rate loop_rate(10.);
    tf::Quaternion q;

    while (ros::ok()){
        // Acquisition de la commande et du GPS
        ros::spinOnce();

        // MàJ de la position
        Matrix4d A;
        A << 1, 0, 0, dt * cos(x0[2]), 0, 1, 0, dt * sin(x0[2]), 0, 0, 1, 0, 0, 0, 0, 1 - dt * abs(x0[3]);
        kalman(x0, Gx0, ukal, Galpha, A, ykal, Gbeta, C);

        // Création et publication du message contenant la position estimée
        // ------------------------------------------------------------------------
        filters::GPose msg;
        msg.east = x0(0);
        msg.north = x0(1);
        msg.vitesse = x0(3); //x, y, theta, v
        msg.heading = x0(2);
        estimated_state_pub.publish(msg);
        // -----------------------------------------------------
        loop_rate.sleep();
    }
    return 0;
}