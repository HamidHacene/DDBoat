#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "std_msgs/Float64MultiArray.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"


#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PointStamped.h"
#include "eigen3/Eigen/Dense"

#include "tf/tf.h"

# define M_PI           3.14159265358979323846  


Eigen::Vector4d Xbateau = {0.0, 0.0, 0.0, 0.0};
Eigen::Vector2d Xcible = {5, 5}; 
Eigen::Vector2d Vcible = {0, 0};
Eigen::Vector2d Acible = {0, 0}; 

Eigen::Vector2d u = {1,1};




void get_X_bateau(const std_msgs::Float64MultiArray msg){
    Xbateau[0] =    msg.data[0]; //x
    Xbateau[1] =     msg.data[1]; //y
    Xbateau[2] =    msg.data[2]; //theta =  - thetaboussole 
    Xbateau[3] =    msg.data[3];  //v

    }

void get_position_cible(const geometry_msgs::Vector3 msg2){
    Xcible[0] =  msg2.x;
    Xcible[1] =  msg2.y;
}

void get_vitesse_cible(const geometry_msgs::Vector3 msg3){
    Vcible[0] =  msg3.x;
    Vcible[1] =  msg3.y;
}

void get_acceleration_cible(const geometry_msgs::Vector3 msg4){
    Acible[0] =  msg4.x;
    Acible[1] =  msg4.y;
}

void controller(){
    double x = Xbateau[0]; 
    double y = Xbateau[1];
    double theta = Xbateau[2];
    double v = Xbateau[3];

    double st = std::sin(theta);
    double ct = std::cos(theta);

    Eigen::Matrix2d A;
    A << ct - v*st, ct + v*st,
         st + v*ct, st - v*ct; 

    Eigen::Vector2d B = {-std::abs(v)*v*ct, -std::abs(v)*v*st};
    Eigen::Vector2d Y = {x, y};
    Eigen::Vector2d dY = {v*ct, v*st};
    Eigen::Vector2d a = {1, 2};
    Eigen::Vector2d b = {2, 3};
    Eigen::Vector2d z = z = 4*(Xcible - Y) + 4*(Vcible - dY);
    // z = 2*(w - Y) + 2*(dw - dY);
    //z = kp*(w - Y) + kd*(dw - dY);
    u = A.fullPivLu().solve(z - B);
}
    


   
int main(int argc, char **argv){

    // INITIALISATION
    ros::init(argc, argv, "control");

    //DECLARATION DE SUBSCRIBERS
    ros::NodeHandle xbateau;
    ros::NodeHandle xcible;
    ros::NodeHandle vcible;
    ros::NodeHandle acible;
    ros::Subscriber sub1 = xbateau.subscribe("X_bateau", 1000, get_X_bateau);
    ros::Subscriber sub2 = xcible.subscribe("position_cible", 1000, get_position_cible);
    ros::Subscriber sub3 = vcible.subscribe("vitesse_cible", 1000, get_vitesse_cible);
    ros::Subscriber sub4 = vcible.subscribe("acceleration_cible", 1000, get_acceleration_cible);

    //DECLARATION DE PUBLISHERS
    
    ros::NodeHandle commande;
    ros::Publisher pub = commande.advertise<std_msgs::Float64MultiArray>("command", 1000);


    
    ros::Rate loop_rate(25);
    while (ros::ok()){
        
        ros::spinOnce();
        controller();
        std_msgs::Float64MultiArray com;
         com.data.clear();
        std::vector<double> X_control = {u[0], u[1]};
        com.data.insert(com.data.end(), X_control.begin(), X_control.end());

        // PUBLICATION
        pub.publish(com);
        
        loop_rate.sleep();

    }

    }
