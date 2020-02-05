#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include "tf/tf.h"
#include "tf/tf.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "eigen3/Eigen/Dense"
#include "std_msgs/Float64MultiArray.h"



Eigen::Vector2d u ={0.0, 0.0};
Eigen::Vector4d X = {0.0, 0.0, 0.0, 0.0};
double dt;
double v;


void intergration_euler(){ 
    Eigen::Vector4d dX = {X[3]*cos(X[2]), X[3]*sin(X[2]), u[0] - u[1], u[0] + u[1] - abs(X[3])*X[3]};
    X = X + dt*dX;

}

void get_u(const std_msgs::Float64MultiArray msg){
    u[0] =  msg.data[0];
    u[1] =  msg.data[1];
    }



int main(int argc, char **argv){
    ros::init(argc, argv, "simulator");



    ros::NodeHandle commande;;
    ros::NodeHandle xbateau;
    ros::NodeHandle xbateau_visualisation;

    ros::Publisher pub = xbateau.advertise<std_msgs::Float64MultiArray>("X_bateau", 1000);
    ros::Publisher pub_visualisation = xbateau_visualisation.advertise<geometry_msgs::PoseStamped>("X_bateau_visu", 1000);


    ros::Subscriber sub = commande.subscribe("command", 1000, get_u);




 
    dt = 0.04;
    ros::Rate loop_rate(25);

 
    while (ros::ok()){

        //   SIMULATION 

        ros::spinOnce();
        intergration_euler();

        std_msgs::Float64MultiArray msg;
        geometry_msgs::PoseStamped vect;
        
         msg.data.clear();
        std::vector<double> Xv = {X[0], X[1], X[2], X[3]};
        msg.data.insert(msg.data.end(), Xv.begin(), Xv.end());

        //VISUALISATION

        vect.header.stamp = ros::Time::now();
        vect.header.frame_id = "map";

        vect.pose.position.x = X[0];
        vect.pose.position.y = X[1];
        vect.pose.position.z = 0;
        tf::Quaternion q;
        q.setRPY(0,0,X[2]);
        tf::quaternionTFToMsg(q, vect.pose.orientation);

        //PUBLICATION 

        pub.publish(msg);
        pub_visualisation.publish(vect);
        loop_rate.sleep();
    }

}
