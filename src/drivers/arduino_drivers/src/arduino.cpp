#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>

#include "serial/serial.h"
#include "ros/ros.h"

#include "arduino_drivers/Motor_dual.h"

using namespace std;

#define RATE 10

string port = "/dev/ttyACM0";
int baudrate = 115200;
serial::Serial arduino(port, baudrate, serial::Timeout::simpleTimeout(1000));
double cmd_l, cmd_r;

int cast_cmd(int cmd);
void send_arduino_motor_cmd(int cmdl, int cmdr);
void commandeCallback(const arduino_drivers::Motor_dual::ConstPtr &msg);

// Attention à bien inclure chaque type de message !
int main(int argc, char **argv)
{
    // Initialisation du node : le troisième argument est son nom
   
    cout << "Ouverture de la connection" << endl;
    if (arduino.isOpen())
    {
        cout << "Serial -> OK" << endl;
        usleep(100 * 1000);
        string data = arduino.readline();
        cout << "Arduino init status -> " << data << endl;
    }
    else
    {
        cout << "** erreur arduino driver **" << endl;
        return 0;
    }

    ros::init(argc, argv, "arduino_driver");
    ros::NodeHandle n;
    ros::Rate loop_rate(RATE);

    ros::Subscriber sub = n.subscribe("controlled_cmd_motor", 1000, commandeCallback);


    cout << "-> Lancement du driver Arduino" << endl;

    while (ros::ok())
    {
        ros::spinOnce();

        send_arduino_motor_cmd(cmd_l, cmd_r);

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