#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>

#include "serial/serial.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose2D.h"

using namespace std;

#define RATE 2

string port = "/dev/ttyUSB0";
int baudrate = 115200;
serial::Serial encoder(port, baudrate, serial::Timeout::simpleTimeout(1000));

int cast_cmd(int cmd);
void send_arduino_motor_cmd(int cmdl, int cmdr);
//void get_encoders_data(geometry_msgs::Pose2D &data);
void get_encoders_data();

// Attention à bien inclure chaque type de message !
int main(int argc, char **argv)
{
    // Initialisation du node : le troisième argument est son nom

    cout << "Ouverture de la connection" << endl;
    if (encoder.isOpen())
    {
        cout << "Serial -> OK" << endl;
        usleep(100 * 1000);
    }
    else
    {
        cout << "** erreur encoder driver **" << endl;
        return 0;
    }

    ros::init(argc, argv, "encoder_driver");
    ros::NodeHandle n;
    ros::Rate loop_rate(RATE);

    ros::Publisher encoder_pub = n.advertise<geometry_msgs::Pose2D>("encoders", 0);

    cout << "-> Lancement du driver Encoder" << endl;

    geometry_msgs::Pose2D encoders_data;

    while (ros::ok())
    {

        get_encoders_data();
        //encoder_pub.publish(encoder_data);

        ros::spinOnce();
        // Pause
        loop_rate.sleep();
    }

    return 0;
}

void get_encoders_data()
{
    bool sync = true;
    string data;
    string v = encoder.read(17);
    //cout << "encoders values : " << v << endl;

    char c1 = v[0];
    char c2 = v[1];

    int sensLeft;
    int sensRight;
    int posLeft;
    int posRight;
    int voltLeft;
    int voltRight;

    if (c1 != 0xff || c2 != 0x0d)
    {
        cout << "sync lost, exit" << endl;
        sync = false;
    }
    else
    {
        sensLeft = (int)v[6];
        sensRight = (int)v[7];
        posLeft = (int)(v[8] << 8);
        posLeft = posLeft + (int)v[9];
        posRight = (int)(v[10] << 8);
        posRight = posRight + (int)v[11];
        voltLeft = (int)(v[12] << 8);
        voltLeft = voltLeft + (int)v[13];
        voltRight = (int)(v[14] << 8);
        voltRight = voltRight + (int)v[15];
    }

    cout << "values -> " << posLeft << endl;
}