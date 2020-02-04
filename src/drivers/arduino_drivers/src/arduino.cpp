#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>

#include "serial/serial.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

using namespace std;

string port = "/dev/ttyACM0";
int baudrate = 115200;

void sleep(unsigned long miliseconds);
int cast_cmd(int cmd);

// Attention à bien inclure chaque type de message !
int main(int argc, char **argv)
{
    // Initialisation du node : le troisième argument est son nom
    serial::Serial arduino(port, baudrate, serial::Timeout::simpleTimeout(1000));
    cout << "Is the serial port open?";
    if (arduino.isOpen())
    {
        cout << " Yes." << endl;
    }
    else
    {
        cout << " No." << endl;
    }
        

    ros::NodeHandle n;

    return 0;
}

void sleep(unsigned long miliseconds)
{
    usleep(miliseconds * 1000);
}


int cast_cmd(int cmd)
{
    if(cmd > 255)
    {
        return 255; 
    }
    else if(cmd < 0)
    {
        return 0;
    }
}