#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/TwistStamped.h>

#include "serial_port.h"
#include "qbot_interface.h"

int main(int argc, char **argv);
void commands(Qbot_Interface &api);
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);


// quit handler
Qbot_Interface *qbot_interface_quit;
Serial_Port *serial_port_quit;
void quit_handler( int sig );

