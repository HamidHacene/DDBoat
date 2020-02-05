#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/Vector3.h"
#include <eigen3/Eigen/Dense>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

# define M_PI           3.14159265358979323846  

using namespace std;
using namespace Eigen;

Vector3d ykal;
Vector4d ukal;

void kalman_predict(Vector4d& x1, Matrix4d& Gx1, Vector4d& xup, Matrix4d& Gup, Vector4d& u, Matrix4d& Galpha, Matrix4d& A)
{
    Gx1 = A * Gup * A.transpose() + Galpha;
    x1 = A * xup + u;
}

void kalman_correct(Vector4d&xup, Matrix4d& Gup, Vector4d& x0, Matrix4d& Gx0, Vector3d& y, Matrix3d& Gbeta, MatrixXd& C)
{
    MatrixXd S = C * Gx0 * C.transpose() + Gbeta;
    MatrixXd K = Gx0 * C.transpose() * S.inverse();
    VectorXd ytilde = y - C * x0;
    Gup = (MatrixXd::Identity(x0.size(), x0.size()) - K * C) * Gx0;
    xup = x0 + K * ytilde;
}

void kalman(Vector4d& x0, Matrix4d& Gx0, Vector4d& u, Matrix4d& Galpha, Matrix4d& A, Vector3d& y, Matrix3d& Gbeta, MatrixXd& C)
{
    Vector4d xup;
    Matrix4d Gup;
    kalman_correct(xup, Gup, x0, Gx0, y, Gbeta, C);
    kalman_predict(x0, Gx0, xup, Gup, u, Galpha, A);
}

void commandCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    ukal << 0., 0., msg->data[0], msg->data[1];
}

void lambertCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    ykal(0) = msg->data[1];  //east
    ykal(1) = msg->data[0];  //north
    // ykal = (x, y, thetaBoussole)
}

void capCallback(const std_msgs::Float64::ConstPtr& msg)
{
    double thetaB = msg->data;
    ykal(2) = -thetaB - 90;
}

int main(int argc, char **argv)
{
    //Initialisation Kalman
    // ------------------------------------------
    Vector4d x0 = {40.0, 0.0, 0.0, 0.1}; // doit être initialisé correctement dans le launch !
    Matrix4d Gx0 = 10 * MatrixXd::Identity(4, 4);
    Matrix4d Galpha = MatrixXd::Zero(4, 4);
    MatrixXd C(3, 4);
    C << 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.;
    Matrix2d B;
    B << 1., -1., 1., 1.;
    Matrix3d Gbeta;
    Gbeta << 5., 0., 0., 0., 5., 0., 0., 0., 5. * M_PI / 180.;
    // ------------------------------------------

    const double dt = 0.1;
    ros::init(argc, argv, "kalman");
    ros::NodeHandle n;
    n.param<double>("pos_x", x0(0), 40.0);
    n.param<double>("pos_y", x0(1), 0.0);
    n.param<double>("yaw", x0(2), 0.0);
    n.param<double>("yaw", x0(3), 0.1);

    //Abonnement aux topics
    ros::Publisher estimated_state_pub = n.advertise<std_msgs::Float64MultiArray>("poseCorrected", 1000);
    ros::Subscriber com_sub = n.subscribe("command", 1000, commandCallback);
    ros::Subscriber lamb_sub = n.subscribe("poseRaw", 1000, lambertCallback);
    ros::Subscriber cap_sub = n.subscribe("capFiltered", 1000, capCallback);
    ros::Rate loop_rate(10.);

    while (ros::ok())
    {
        // Acquisition de la commande et du GPS
        ros::spinOnce();

        // MàJ de la position
        Matrix4d A;
        A << 1., 0., 0., dt * cos(x0[2]), 0., 1., 0., dt*sin(x0[2]), 0., 0., 1., 0., 0., 0., 0., 1. - dt*abs(x0[3]);
        kalman(x0, Gx0, ukal, Galpha, A, ykal, Gbeta, C);

        // Création et publication du message contenant la position estimée
        // ------------------------------------------------------------------------
        std_msgs::Float64MultiArray msg;
        msg.data.clear();
        //east, north, cap, vitesse
        std::vector<double> Xhat = {x0(0), x0(1), x0(2)*M_PI/180. , x0(3)};
        /*message publié
            data(0) = east --> x
            data(1) = north --> y
            date(2) = thetaRadian
            data(3) = vitesse m/s
        */
        msg.data.insert(msg.data.end(), Xhat.begin(), Xhat.end());

        estimated_state_pub.publish(msg);
        // -----------------------------------------------------
        loop_rate.sleep();
    }
    return 0;
}
