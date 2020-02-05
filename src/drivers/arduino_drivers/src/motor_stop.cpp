#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>

#include "serial/serial.h"

using namespace std;

#define RATE 10

string port = "/dev/ttyACM0";
int baudrate = 115200;
serial::Serial arduino(port, baudrate, serial::Timeout::simpleTimeout(1000));
double cmd_l, cmd_r;

int cast_cmd(int cmd);
void send_arduino_motor_cmd(int cmdl, int cmdr);

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



    cout << "->Stop Motor" << endl;

    while (true)
    {

        send_arduino_motor_cmd(0, 0);

        // Pause
        sleep(0.1);
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

    char cmd_str[50];
    sprintf(cmd_str, "M %3.3d %3.3d;", cmdl, cmdr);
    arduino.write(cmd_str);
    cout << cmd_str << endl;
}
