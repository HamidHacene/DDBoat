#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PointStamped.h"

# define M_PI           3.14159265358979323846  


double Xcible[2]; //coordonnes de la cible
double Vcible[2]; //vitesse de la cible 
double Acible[2]; //accélération de la cible 
double t;



void cible(){

    //POINT FIXE


     Xcible[0] =  10;
    Xcible[1] = 10;
    Vcible[0] = 0;
    Vcible[1] = 0;
    Acible[0] =0;
    Acible[1] = 0;

/*

    //POINT MOBILE 
    double m= 5;
    double s = 5;
    Xcible[0] =  m*sin(t/s) ;
    Xcible[1] = m*sin(2*t/s);
    Vcible[0] = m*cos(t/s)/s;
    Vcible[1] = m*2*cos(2*t/s)/s;
 
*/
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




    ros::Rate loop_rate(10);
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
