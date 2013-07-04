/*
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2013, Agustin Caverzasi, Fernando Saravia Rajal,
 * Laboratorio de Arquitectura de Computadoras (LAC), 
 * Grupo de Robotica y Sistemas Integrados (GRSI),
 * Universidad Nacional de Córdoba (UNC), Cordoba, Argentina. 
 * Original file from 
 * https://raw.github.com/ros/ros_tutorials/groovy-devel/roscpp_tutorials/listener/listener.cpp
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
 * This node runs under on-board robot computer and recive commands from some of two sources: 
 * 'wifi_talker' node in remote computer for teleoperation mode, or 'keyboard' node in on-board
 * robot computer for debugging mode (testing).
 * For more information please see https://github.com/agucaverzasi/Robot-movil-mapeo3D/blob/master/Informe.pdf
 */ 

#include "ros/ros.h"
#include "std_msgs/String.h"

ros::Publisher pub;
ros::Subscriber sub;

/* 
 * Recive commands from other node suscribed to 'commands' topic
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Server recived: [%s]", msg->data.c_str());
  
  pub.publish(msg);
  ros::spinOnce();
}

int main(int argc, char **argv)
{
  // 'wifi_listener' node initialization
  ros::init(argc, argv, "wifi_listener");
  ros::NodeHandle n;
  // 'wifi_listener'node suscription to 'commands' topic 
  pub = n.advertise<std_msgs::String>("commands", 1000);
  sub = n.subscribe("wifi", 1000, chatterCallback);
  ros::spin();

  return 0;
}
