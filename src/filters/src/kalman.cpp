#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/Vector3.h"
#include <eigen3/Eigen/Dense>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

#define M_PI 3.14159265358979323846

using namespace std;
using namespace Eigen;

Vector2d ykal;
double speed;
double heading;
double east;
double north;

void kalman_predict(Vector2d &x1, Matrix2d &Gx1, Vector2d &xup, Matrix2d &Gup, Vector2d &u, Matrix2d &Galpha, Matrix2d &A)
{
    Gx1 = A * Gup * A.transpose() + Galpha;
    x1 = A * xup + u;
}

void kalman_correct(Vector2d &xup, Matrix2d &Gup, Vector2d &x0, Matrix2d &Gx0, Vector2d &y, Matrix2d &Gbeta, MatrixXd &C)
{
    MatrixXd S = C * Gx0 * C.transpose() + Gbeta;
    MatrixXd K = Gx0 * C.transpose() * S.inverse();
    VectorXd ytilde = y - C * x0;
    Gup = (MatrixXd::Identity(x0.size(), x0.size()) - K * C) * Gx0;
    xup = x0 + K * ytilde;
}

void kalman(Vector2d &x0, Matrix2d &Gx0, Vector2d &u, Matrix2d &Galpha, Matrix2d &A, Vector2d &y, Matrix2d &Gbeta, MatrixXd &C)
{
    Vector2d xup;
    Matrix2d Gup;
    kalman_correct(xup, Gup, x0, Gx0, y, Gbeta, C);
    kalman_predict(x0, Gx0, xup, Gup, u, Galpha, A);
}

void lambertCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    //ykal(0) = msg->data[1]; //east
    //ykal(1) = msg->data[0]; //north
    east = msg->data[1];
    north = msg->data[0];
    speed = msg->data[2];
    // ykal = (x, y, thetaBoussole)
}

void capCallback(const std_msgs::Float64::ConstPtr &msg)
{
    double thetaB = msg->data;
    heading = (90 - thetaB)*M_PI/180.;
}

int main(int argc, char **argv)
{
    //Initialisation Kalman
    // ------------------------------------------
    Vector2d x0 = {40.0, 0.0}; // x0 = {x0, y0}
    Matrix2d Gx0 = 10 * MatrixXd::Identity(2, 2);
    Matrix2d Galpha = 100 * MatrixXd::Identity(2, 2);
    MatrixXd C = MatrixXd::Identity(2, 2);
    Matrix2d Gbeta = 25 * MatrixXd::Identity(2, 2);
    Matrix2d A = MatrixXd::Identity(2, 2);
    Vector2d U = {0., 0.};
    // ------------------------------------------

    const double dt = 0.1;
    ros::init(argc, argv, "kalman");
    ros::NodeHandle n;
    n.param<double>("pos_x", x0(0), 40.0);
    n.param<double>("pos_y", x0(1), 0.0);

    //Abonnement aux topics
    ros::Publisher estimated_state_pub = n.advertise<std_msgs::Float64MultiArray>("poseCorrected", 1000);
    ros::Subscriber lamb_sub = n.subscribe("poseRaw", 1000, lambertCallback);
    ros::Subscriber cap_sub = n.subscribe("capFiltered", 1000, capCallback);
    ros::Rate loop_rate(15.);

    while (ros::ok())
    {
        // Acquisition de la commande et du GPS
        ros::spinOnce();
        //U(0) = speed*cos(heading);
        //U(1) = speed*sin(heading);
        // MàJ de la position
        //kalman(x0, Gx0, U, Galpha, A, ykal, Gbeta, C);

        // Création et publication du message contenant la position estimée
        // ------------------------------------------------------------------------
        std_msgs::Float64MultiArray msg;
        msg.data.clear();
        //east, north, cap, vitesse
        //std::vector<double> Xhat = {x0(0), x0(1), heading, speed};
        std::vector<double> Xhat = {east, north, heading, speed};
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
