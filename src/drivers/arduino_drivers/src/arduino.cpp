#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>

#include "serial/serial.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

using namespace std;

#define RATE 2

string port = "/dev/ttyACM0";
int baudrate = 115200;
serial::Serial arduino(port, baudrate, serial::Timeout::simpleTimeout(1000));


int cast_cmd(int cmd);
void send_arduino_motor_cmd(int cmdl, int cmdr);

// Attention à bien inclure chaque type de message !
int main(int argc, char **argv)
{
    // Initialisation du node : le troisième argument est son nom
   
    cout << "Ouverture de la connection" << endl;
    if (arduino.isOpen())
    {
        cout << "OK" << endl;
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

    cout << "-> Lancement du driver Arduino" << endl;

    while (ros::ok())
    {

        send_arduino_motor_cmd(100, 100);

        ros::spinOnce();
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