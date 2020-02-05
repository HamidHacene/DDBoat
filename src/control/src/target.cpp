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


double Xcible[2]; //coordonnes de la cible
double Vcible[2]; //vitesse de la cible 
double Acible[2]; //accélération de la cible 
double t;



void cible(){

    //TO DO
    double m= 1;
    Xcible[0] =  m*sin(t) ;
    Xcible[1] = m*sin(2*t);
    Vcible[0] = m*cos(t);
    Vcible[1] = m*2*cos(2*t);
    Acible[0] = -m*sin(t);
    Acible[1] = -4*m*sin(2*t);

}






int main(int argc, char **argv){
    ros::init(argc, argv, "target");


    //PUBLISHERS
    ros::NodeHandle xcible;
    ros::NodeHandle vcible;
    ros::NodeHandle xcible_affichage;
    ros::NodeHandle acible;
    
    ros::Publisher pub1 = xcible.advertise<geometry_msgs::Vector3>("position_cible", 1000);
    ros::Publisher pub2 = vcible.advertise<geometry_msgs::Vector3>("vitesse_cible", 1000);
    ros::Publisher pub4 = vcible.advertise<geometry_msgs::Vector3>("acceleration_cible", 1000);
    ros::Publisher pub3 = vcible.advertise<geometry_msgs::PointStamped>("Point_cible", 1000);




    ros::Rate loop_rate(25);
    while (ros::ok()){
        ros::spinOnce();

        geometry_msgs::Vector3 pos_cible;
        geometry_msgs::Vector3 vit_cible;
        geometry_msgs::Vector3 acc_cible;
        geometry_msgs::PointStamped point_cible;


        t= ros::Time::now().toSec() ;
        cible();
        pos_cible.x = Xcible[0];
        pos_cible.y = Xcible[1];
        vit_cible.x = Vcible[0];
        vit_cible.y = Vcible[1];
        acc_cible.x = Acible[0];
        acc_cible.y = Acible[1];




        //POUR L'AFFICHAGE DE LA CIBLE SUR RVIZ

        point_cible.header.stamp = ros::Time::now() ;
        point_cible.header.frame_id = "map";
        point_cible.point.x = Xcible[0];
        point_cible.point.y = Xcible[1];
        point_cible.point.z = 0;


        //PUBLICATION CIBLE

        pub1.publish(pos_cible);
        pub2.publish(vit_cible);
        pub4.publish(acc_cible);
        pub3.publish(point_cible);
      

        loop_rate.sleep();

    }







}
