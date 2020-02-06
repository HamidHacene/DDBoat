#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "eigen3/Eigen/Dense"
#include "arduino_drivers/Motor_dual.h"

# define M_PI           3.14159265358979323846 


Eigen::Vector2d u = {1,1};
Eigen::Vector2d v = {0,0};

double vitesse_droite;
double vitesse_gauche;

void conversion(){
    double K = 1;
    v[0] = u[0]*(255/8) + (255/2);
    v[1] = u[1]*(255/8) + (255/2);
    
}

void get_command(const std_msgs::Float64MultiArray msg){
    u[0] =  msg.data[0];
    u[1] =  msg.data[1];
}



int main(int argc, char **argv){

    // INITIALISATION
    ros::init(argc, argv, "motor_input");

    //DECLARATION DE SUBSCRIBERS

     ros::NodeHandle commande;
    ros::Subscriber sub1 = commande.subscribe("command", 1000, get_command);

    //DECLARATION DE PUBLISHERS

    ros::NodeHandle input;
    ros::Publisher pub = input.advertise<arduino_drivers::Motor_dual>("motor_cmd_2", 1000);

 
    ros::Rate loop_rate(10);
    while (ros::ok()){

        ros::spinOnce();
        arduino_drivers::Motor_dual motor_input;
        conversion();

        motor_input.left = v[0];
        motor_input.right = v[1];
        pub.publish(motor_input);




        loop_rate.sleep();

    }

    }
