/*
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2013, Agustin Caverzasi, Fernando Saravia Rajal,
 * Laboratorio de Arquitectura de Computadoras (LAC), 
 * Grupo de Robotica y Sistemas Integrados (GRSI),
 * Universidad Nacional de Córdoba (UNC), Cordoba, Argentina. 
 * Original file from 
 * https://raw.github.com/ros/ros_tutorials/groovy-devel/roscpp_tutorials/talker/talker.cpp 
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
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
 * This node allows user to send commands from remote computer to on-board robot computer in teleoperation mode.
 * For more information please see https://github.com/agucaverzasi/Robot-movil-mapeo3D/blob/master/Informe.pdf
 */ 

#include "ros/ros.h"
#include <signal.h>
#include "std_msgs/String.h"
#include <termios.h>

using namespace std;

// arrows keys codes
#define KEYCODE_R 		0x43 
#define KEYCODE_L 		0x44
#define KEYCODE_U 		0x41
#define KEYCODE_D 		0x42
#define KEYCODE_Q 		0x71
#define KEYCODE_E 		0x65
#define KEYCODE_SPACE 	0x20

int kfd = 0;
struct termios cooked, raw;
ros::Publisher pub;				

/*
 * Publish command through 'wifi' ROS topic
 */
void commandSend(std::string command)
{
	std_msgs::String msg;
    msg.data = command;
    ROS_INFO("Client sent: [%s]", msg.data.c_str());	//print debugging info
	pub.publish(msg);									//publish message
}

/*
 * Get the full command typed from console by user in 'commands' input mode
 */
void getWord()
{
	ROS_INFO("Input commands 'up', 'down', 'left', 'right', or 'stop' to move the Robot. Type 'quit' to change input mode");
	for(;;)
	{ 
		string in;
		getline(cin,in);
		stringstream input(in);
		if(input.str()=="quit")break;
		else commandSend(input.str());
	}
}

/*
 * Get the arrow key typed by user in 'arrows' input mode 
 */
void getKey()
{
  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  ROS_INFO("Use arrow keys to move the Robot. Type 'q' to change input mode");
  
  for(;;)
  { 
    char c;
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
	if(c=='q')break;
	
	ROS_DEBUG("value: 0x%02X\n", c);
    switch(c)
    { 	
	  case KEYCODE_L:
        ROS_DEBUG("LEFT");
        commandSend("left");
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        commandSend("right");
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        commandSend("up");
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        commandSend("down");
        break;
      case KEYCODE_SPACE:
        ROS_DEBUG("STOP");
        commandSend("stop");
        break;
      default:
        break;
    }
  }
  return;
}

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
  // node initialization
  ros::init(argc, argv, "wifi_talker");
  ros::NodeHandle n;
  // 'wifi_talker' node suscription to 'wifi' topic 
  pub = n.advertise<std_msgs::String>("wifi", 1000);
  
  signal(SIGINT,quit);
  
  for(;;)
  {
	ROS_INFO("Reading from keyboard.");
	ROS_INFO("Type 'arrows' for use arrow keys, or 'commands' for input commands from keyboard.");
	// get input mode from console
	string in;
	getline(cin,in);
	stringstream input(in);
	// select input mode, typing keys with arrows or writing full command 	
	if(input.str()=="arrows")getKey();
	  else if(input.str()=="commands")getWord();
  }
  
  return 0;
}


