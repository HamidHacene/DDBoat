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
#include "arduino_driver/Motor_dual.h"

#include "tf/tf.h"

# define M_PI           3.14159265358979323846 
/*

Eigen::Vector2d u = {1,1};
Eigen::Vector2d v = {0,0};

double vitesse_droite;
double vitesse_gauche;

void conversion(){
    
}
*/


int main(int argc, char **argv){

    // INITIALISATION
    ros::init(argc, argv, "motor_input");

    //DECLARATION DE SUBSCRIBERS

 
    ros::Rate loop_rate(25);
    while (ros::ok()){
        ros::spinOnce();


        loop_rate.sleep();

    }

    }
