#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf/tf.h"

#include "eigen3/Eigen/Dense"

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
    ros::Subscriber sub1 = xbateau.subscribe("poseCorrected", 1000, get_X_bateau);
    ros::Subscriber sub2 = xcible.subscribe("position_cible", 1000, get_position_cible);
    ros::Subscriber sub3 = vcible.subscribe("vitesse_cible", 1000, get_vitesse_cible);
    ros::Subscriber sub4 = vcible.subscribe("acceleration_cible", 1000, get_acceleration_cible);

    //DECLARATION DE PUBLISHERS
    
    ros::NodeHandle commande;
    ros::NodeHandle xbateau_visu;
    ros::NodeHandle node_handle;

    ros::Publisher pub = commande.advertise<std_msgs::Float64MultiArray>("command", 1000);
    ros::Publisher pub2 = xbateau_visu.advertise<geometry_msgs::PoseStamped>("X_bateau_visu", 1000);
    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "boat";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;

    
    ros::Rate loop_rate(10);
    while (ros::ok()){
        
        ros::spinOnce();
        controller();
        geometry_msgs::PoseStamped pose;
        std_msgs::Float64MultiArray com;
        com.data.clear();
        std::vector<double> X_control = {u[0], u[1]};
        com.data.insert(com.data.end(), X_control.begin(), X_control.end());

        //header
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        //position
        pose.pose.position.x = Xbateau[0];
        pose.pose.position.y = Xbateau[1];
        pose.pose.position.z = 0;
        tf::Quaternion q;
        q.setRPY(0,0,Xbateau[2]);
        tf::quaternionTFToMsg(q, pose.pose.orientation);
        //3D

        marker.header.stamp = ros::Time();

        marker.pose = pose.pose;
        double scale = 0.4;
        marker.scale.x = scale*1;
        marker.scale.y = scale*1;
        marker.scale.z = scale*1;
        marker.color.a = 1.0; // alpha = transparence
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.mesh_resource = "package://control/meshs/boat.dae";


        vis_pub.publish(marker);

        // PUBLICATION
        pub.publish(com);
        pub2.publish(pose);

        ROS_INFO("position x : %f", Xbateau[0]);
        ROS_INFO("position  y: %f", Xbateau[1]);
        ROS_INFO(" angle theta : %f", Xbateau[2]);
        ROS_INFO(" vitesse : %f",Xbateau[3]);
        
        loop_rate.sleep();

    }

    }
