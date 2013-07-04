/*
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2013, Agustin Caverzasi, Fernando Saravia Rajal,
 * Laboratorio de Arquitectura de Computadoras (LAC), 
 * Grupo de Robotica y Sistemas Integrados (GRSI),
 * Universidad Nacional de Córdoba (UNC), Cordoba, Argentina. 
 * Copyright (C) 2008, Willow Garage, Inc.
 * All rights reserved.
 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

 * Contact information
 * 
 * Agustin Caverzasi, UNC
 * e-mail   :  agucaverzasi@gmail.com
 *  
 * Fernando Saravia Rajal, UNC
 * e-mail   :  fersarr@gmail.com
 *  
 * Updates should always be available at https://github.com/agucaverzasi/Robot-movil-mapeo3D
 * 
 * Changelog:
 *           01/07/2013 file added to repository
 *
 * This aim of this node is to entablish a comunication between ROS (on-board computer) and Arduino.
 * For more information please see https://github.com/agucaverzasi/Robot-movil-mapeo3D/blob/master/Informe.pdf
 */ 


#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <std_msgs/String.h>
#include <fstream>

using namespace std;

#define SIZE 1024

int fd;
char reply[SIZE];

class Robot
{
public:
		Robot();

private:
		ros::NodeHandle nh_;
		ros::Subscriber wifi;
		ros::NodeHandle port;
		void getCallback(const std_msgs::String::ConstPtr& msg);
};

Robot::Robot()
{
	// suscription to 'commands' topic to recive commands
	wifi = nh_.subscribe("commands", 1000, &Robot::getCallback,this);
}

/*
 * Recive a command from other ROS node, and send it to Arduino to get a robot motion
 */
void Robot::getCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Robot recived: [%s]", msg->data.c_str());

	char str[40];
	string s=msg->data.c_str();
	strcpy(str,msg->data.c_str());
	strcat (str,"\n");
	write(fd,str,s.length()+1);
}

void quit(int sig)
{
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
	// node initialization
	ros::init(argc, argv, "robot");
	Robot robot;
	
	// IMPORTANT! file descriptor has to have the same number as Arduino serial monitor to entablish a comunication. 
	// E.g. if Arduino serial monitor is ACM1, file descriptor is "/dev/ttyACM1"
	fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
	if( fd > 0 )
	{
		signal(SIGINT,quit);
		ROS_INFO("Arduino connection established, waiting for initialization...");

		int flag=0;
		read(fd, reply, SIZE);
		char *s;
		while(flag==0)
		{
			s = strstr(reply, "Done");      
	 		if (s != NULL) 
	 		{   
	 			flag=1;        
	 			//when this line is printed the Arduino can start to recive move commands              
	 			ROS_INFO("Arduino initialization done"); 
	 		}
	 		else
		 	{
		 		flag=0;
		 		read(fd, reply, SIZE);
		 		//ROS_INFO("Not found\n"); 
			}
		}
		// call to callBack() function all the time, until ros::shutdown() is called or Ctrl-C is pressed
		ros::spin();	
		close(fd);
	}
	else
	{
		// this can ocurr when Arduino Serial monitor number and the file descriptor are not coincident
		ROS_INFO("No se pudo establecer la conexion con Arduino: no se pudo abrir el device file");
	}
	return 0;
}
