#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>

#include "serial/serial.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "arduino_drivers/Motor_dual.h"

using namespace std;

#define RATE 10

double cmd_l, cmd_r;



// Attention à bien inclure chaque type de message !
int main(int argc, char **argv)
{
    // Initialisation du node : le troisième argument est son nom
   


    ros::init(argc, argv, "motor_test");
    ros::NodeHandle n;
    ros::Rate loop_rate(RATE);

    ros::Publisher pub = n.advertise<arduino_drivers::Motor_dual>("motor_cmd", 0);

    arduino_drivers::Motor_dual motor_cmd;

    cout << "-> Lancement du test moteur" << endl;

    double t;

    while (ros::ok())
    {

        t = ros::Time::now().toSec();
        
        motor_cmd.left = 50 + 100*cos(t/10);
        motor_cmd.right = motor_cmd.left;

        pub.publish(motor_cmd);

        // Pause
        loop_rate.sleep();
    }

    return 0;
}
