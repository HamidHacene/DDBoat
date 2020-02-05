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

    while (ros::ok())
    {
        
        motor_cmd.left = ;
        motor_cmd.right = ;
        
        pub.publish(motor_cmd);

        // Pause
        loop_rate.sleep();
    }

    return 0;
}


int cast_cmd(int cmd)
{
    if (cmd > 255)
    {
        return 255;
    }
    else if (cmd < 0)
    {
        return 0;
    }
}


void send_arduino_motor_cmd(int cmdl, int cmdr)
{
    cmdl = cast_cmd(cmdl);
    cmdr = cast_cmd(cmdr);

    char cmd_str [50];
    sprintf(cmd_str, "M %3.3d %3.3d;", cmdl, cmdr);
    arduino.write(cmd_str);
    cout << cmd_str << endl;
}

void get_arduino_status()
{
    arduino.write("P;");
    bool boucle = true;
    int n = 0;
    string data;
    while(boucle)
    {
        usleep(10 * 1000);
        data = arduino.readline();
        if(data.length() >= 4 || n > 50)
        {
            boucle = false;
        }
        n++;
    }
    cout << "Arduino status -> " << data << endl;
}

void commandeCallback(const arduino_drivers::Motor_dual::ConstPtr &msg)
{
    //cout << "commande incoming ! " << endl;
    cmd_l = msg->left;
    cmd_r = msg->right;
}