#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "arduino_drivers/Motor_dual.h"

#define RATE 25

double value_encoder_left, value_encoder_right;
double value_cmd_left, value_cmd_right;
double error_l, error_r;
double previous_error_left = 0, previous_error_right = 0;
double sum_error_left = 0, sum_error_right = 0;


double Kp, Ki, Kd;

void commandCallback(const arduino_drivers::Motor_dual::ConstPtr &msg)
{
    value_cmd_left = msg->left;
    value_cmd_right = msg->right;
}

void encodersCallback(const arduino_drivers::Motor_dual::ConstPtr &msg)
{
    value_encoder_left = msg->left;
    value_encoder_right = msg->right;
}

void PID(double enc_left, double enc_right, double cmd_left, double cmd_right)
{
    double error_right = cmd_right - enc_right;
    double error_left = cmd_left - enc_left;

    sum_error_left = sum_error_left + error_left;
    sum_error_right = sum_error_right + error_right;

    value_cmd_left = value_cmd_left + Kp * error_left + Kd * (error_left - previous_error_left) / RATE + Ki * sum_error_left;
    value_cmd_right = value_cmd_right + Kp * error_right + Kd * (error_right - previous_error_right) / RATE + Ki * sum_error_right;

    previous_error_left = error_left;
    previous_error_right = error_right;
}

int main(int argc, char **argv)
{

    Kp = 0;
    Ki = 0;
    Kd = 0;

    ros::init(argc, argv, "pid_motor");
    ros::NodeHandle n;
    

    ros::Publisher command_corrected = n.advertise<arduino_drivers::Motor_dual>("controlled_cmd_motor", 1000);
    ros::Subscriber values_command = n.subscribe("motor_cmd", 1000, commandCallback);
    ros::Subscriber values_encoders = n.subscribe("encoders", 1000, encodersCallback);

    ros::Rate loop_rate(RATE);

    arduino_drivers::Motor_dual cmd_corrected;

    while (ros::ok())
    {
        ros::spinOnce();

        //PID(value_encoder_left, value_encoder_right, value_cmd_left, value_cmd_right);

        cmd_corrected.left = value_cmd_left;   //commande moteur gauche après pid
        cmd_corrected.right = value_cmd_right; //commande moteur droite après pid

        //cmd_corrected.left = 100;   //commande moteur gauche après pid
        //cmd_corrected.right = 100; //commande moteur droite après pid


        command_corrected.publish(cmd_corrected);

        loop_rate.sleep();
    }
    return 0;
}