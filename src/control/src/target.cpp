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
#include "geometry_msgs/PointStamped.h"

#include "tf/tf.h"
# define M_PI           3.14159265358979323846  


double Xcible[2]; //coordonnes de la cible
double Vcible[2]; //vitesse de la cible 

void cible(){

    //TO DO
    Xcible[0] = 0;
    Xcible[1] = 0;
    Vcible[0] = 0;
    Vcible[1] = 0;

}






int main(int argc, char **argv){
    ros::init(argc, argv, "target");


    //PUBLISHERS
    ros::NodeHandle xcible;
    ros::NodeHandle vcible;
    ros::Publisher pub1 = xcible.advertise<geometry_msgs::Vector3>("position_cible", 1000);
    ros::Publisher pub2 = vcible.advertise<geometry_msgs::Vector3>("vitesse_cible", 1000);



    ros::Rate loop_rate(25);
    while (ros::ok()){
        ros::spinOnce();

        geometry_msgs::Vector3 pos_cible;
        geometry_msgs::Vector3 vit_cible;
        cible();
        pos_cible.x = Xcible[0];
        pos_cible.y = Xcible[1];
        vit_cible.x = Vcible[0];
        vit_cible.y = Vcible[1];


        pub1.publish(pos_cible);
        pub2.publish(vit_cible);

        loop_rate.sleep();

    }







}
