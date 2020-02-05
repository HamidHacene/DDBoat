#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include <string>
#include <arduino_drivers/Motor_dual.h>

double value_encoder_left, value_encoder_right;
double value_cmd_left, value_cmd_right;
double error_l, error_r;
double previous_error_left = 0, previous_error_right = 0;
double sum_error_left = 0, sum_error_right = 0;
int rate = 25;

void commandCallback(const arduino_drivers::Motor_dual& msg){
    value_cmd_left << msg->left;
    value_cmd_right << msg->right;
}

void encodersCallback(const arduino_drivers::Motor_dual& msg){
    value_encoder_left << msg->left;
    value_encoder_right << msg->right;
}

void PID(double enc_left, double enc_right, double cmd_left, double cmd_right, double Kp, double Ki, double Kd){
    double error_right = cmd_right - enc_right;
    double error_left = cmd_left - enc_left;

    sum_error_left = sum_error_left + error_left;
    sum_error_right = sum_error_right + error_right;

    value_cmd_left =  value_cmd_left + Kp * error_left + Kd*(error_left-previous_error_left)/rate + Ki*sum_error_left;
    value_cmd_right = value_cmd_right + Kp * error_right + Kd*(error_right-previous_error_right)/rate + Ki*sum_error_right;

    previous_error_left = error_left;
    previous_error_right = error_right;

}

int main(int argc, char **argv) {

    
    node_priv.param<double>("Kp", Kp_, 1.0);
    node_priv.param<double>("Ki", Ki_, 0.0);
    node_priv.param<double>("Kd", Kd_, 0.0);

    ros::init(argc, argv, "pid_encoders");
    ros::NodeHandle n;
    ros::NodeHandle node_priv("~");
    ros::Rate loop_rate(rate);

    ros::Publisher command_corrected = n.advertise<arduino_drivers::Motor_dual>("command_LR", 1000);
    ros::Subscriber values_command = n.subscribe("motor_cmd", 1000, commandCallback);
    ros::Subscriber values_encoders = n.subscribe("encoders", 1000, encodersCallback);

    while (ros::ok()){
        ros::spinOnce();

        PID(value_encoder_left, value_encoder_right, value_cmd_left, value_cmd_right, Kp_, Ki_ Kd_)

        arduino_drivers::Motor_dual cmd_corrected;
        cmd_corrected.left = value_cmd_left; //commande moteur gauche après pid
        cmd_corrected.right = value_cmd_right; //commande moteur droite après pid
        command_corrected.publish(cmd_corrected);

        loop_rate.sleep();
    }
    return 0;
    

}