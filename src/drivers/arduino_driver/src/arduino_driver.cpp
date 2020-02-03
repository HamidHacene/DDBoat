#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>

#include "serial/serial.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

using namespace std;

String port = "/dev/ttyACM0";
int baudrate = 115200;

void sleep(unsigned long miliseconds);
serial::Serial init_arduino_line();

// Attention à bien inclure chaque type de message !
int main(int argc, char **argv)
{
    // Initialisation du node : le troisième argument est son nom
    ros::init(argc, argv, "boat");
    // Connexion au master et initialisation du NodeHandle qui permet d’avoir accès aux topics et services

    ros::NodeHandle n;
    // Création du publisher avec
    // - le type du message
    // - le nom du topic
    // - la taille du buffer de message à conserver en cas de surchage

    //ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("cap", 1000);
    // La durée de la pause (voir le sleep)
    //ros::Rate loop_rate(10);
    // Boucle tant que le master existe (ros::ok())
    /*
    while (ros::ok())
    {
        // création d’un message de type String
        std_msgs::Float64 msg;
        // affectation la valeur "hello" au champ data
        msg.data = sin(ros::Time::now().toSec())*180;
        // publication du message
        chatter_pub.publish(msg);
        // fonction utile seulement dans le cas de l’utilisation d’un subscriver ou d’un server
        ros::spinOnce();
        // Pause
        loop_rate.sleep();
        // Il est également possible d’utiliser des Timers qui fonctionnent par interruption
        // http://wiki.ros.org/roscpp_tutorials/Tutorials/Timers
    }*/
    return 0;
}

void sleep(unsigned long miliseconds)
{
    usleep(miliseconds * 1000);
}

serial::Serial init_arduino_line()
{
    serial::Serial arduino(port, baudrate, serial::Timeout::simpleTimeout(1000));
    cout << "Is the serial port open?";
    if (my_serial.isOpen())
        cout << " Yes." << endl;
    else
        cout << " No." << endl;
    return arduino;
}

int cast_cmd(int cmd, int cmdmin=0, int cmdmax=255)
{
    if(cmd > cmdmax)
    {
        return cmdmax; 
    }
    else if(cmd < cmdmin)
    {
        return cmdmin;
    }
}