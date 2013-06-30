/* 
 * ====================================================================
 * Copyright (C) 2013, Agustin Caverzasi, Fernando Saravia Rajal,
 * Laboratorio de Arquitectura de Computadoras (LAC), 
 * Grupo de Robotica y Sistemas Integrados (GRSI),
 * Universidad Nacional de Córdoba (UNC), Cordoba, Argentina. 
 * All rights reserved.
 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU GPL v3, General Public License 3 as 
 * published by the Free Software Foundation.
 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * ====================================================================
 
 * Contact information
 
 * Agustin Caverzasi, UNC
 * e-mail   :  agucaverzasi@gmail.com
 
 * Fernando Saravia Rajal, UNC
 * e-mail   :  fersarr@gmail.com
 
 * Updates should always be available at https://github.com/agucaverzasi/Robot-movil-mapeo3D
 *
 * Changelog:
 *           27/06/2013 file added to repository
 */
 
/*
 * Communication between Robot with microcontroller Arduino Mega2560 
 * and Ubuntu with ROS. Commands can recive are: 
 * up, down, right, left,up(amount), down(amount), right(angle), down(angle) 
 * where:
 *       amount->in [cm] for linear movement
 *       right->in [°](degrees between 0°-359°) for rotational movement
 */ 
 
#define COMMANDS
#include "commands.h"
#include "motor.h"
#include "control.h"
#include "localization.h"
#include <QueueList.h>

float new_x=10,new_y=10,new_heading=10, x_aux=0, y_aux=0, heading_aux=0;

/* 
 * When a new command is available (sent from ROS node), it is pushed
 * into a FIFO queue.
 * @param a queue of Strings commands
 */
int getCommand(QueueList<String> &q)  {
  char character;
  if(Serial.available())
  {
      character = Serial.read();
      content+=character;
      if (character == '\n')
        {
            Serial.print("Arduino push: ");
            Serial.print(content);
            q.push(content);
			content = ""; // Clear recieved buffer
        }
  }
}

/* 
 * Get the next command from buffer and pop it
 * @param a queue of Strings commands
 */
void nextCommand(QueueList<String> &q)
{
	if(!q.isEmpty())
    {
    	String c=q.pop();
    	Serial.print("Pop from buffer: "); Serial.println(c);
    	parseCommand(c); 	
	}
}

/*
 * Parse command. 
 * Get the command parameter with get_param() and figure out the x,y coordinates 
 * for lineal movements, or figure out the new heading for rotational movements. 
 * Then we can passing them as argument to the move function call, and stop the 
 * robot at the desired position (given by x,y or heading). 
 * @param command to be parsed
 */
void parseCommand(String command)
{
	  if (command.substring(0,2)=="up")
	  {		
		if(command.substring(0,3)=="up(")
		{
			int dist=get_param(command,3);
			x_aux=dist*cos(getHeadingRad());
			y_aux=dist*sin(getHeadingRad());
			new_x=x+x_aux;
			new_y=y+y_aux;
			up(new_x,new_y);
			
		}
		else 
		{
			x_aux=10*cos(getHeadingRad());
			y_aux=10*sin(getHeadingRad());
			/*
			Serial.print("ang: ");Serial.println(getHeadingRad());
			Serial.print("x*cos: ");Serial.println(x_aux);
			Serial.print("y*sin: ");Serial.println(y_aux);
			*/
			new_x=x+x_aux;
			new_y=y+y_aux;
			up(new_x,new_y);
		}
	  
	  }
	  else if (command.substring(0,4)=="down")
	  {
		if(command.substring(0,5)=="down(")
		{
			int dist=get_param(command,5);
			x_aux=dist*cos(getHeadingRad());
			y_aux=dist*sin(getHeadingRad());
			/*
			Serial.print("ang: ");Serial.println(getHeadingRad());
			Serial.print("x*cos: ");Serial.println(x_aux);
			Serial.print("y*sin: ");Serial.println(y_aux);
			*/
			new_x=x-x_aux;
			new_y=y-y_aux;
			down(new_x,new_y);
		}
		else
		{
			x_aux=10*cos(getHeadingRad());
			y_aux=10*sin(getHeadingRad());
			/*
			Serial.print("ang: ");Serial.println(getHeadingRad());
			Serial.print("x*cos: ");Serial.println(x_aux);
			Serial.print("y*sin: ");Serial.println(y_aux);
			*/
			new_x=x-x_aux;
			new_y=y-y_aux;
			down(new_x,new_y);
		}
	
	  }
	  else if (command.substring(0,5)=="right")
	  {
	  	if(command.substring(0,6)=="right(")
		{
			heading_aux=get_param(command,6);
			new_heading= ((int)( heading - heading_aux )) % 360;
			if(new_heading<0) new_heading=(360+new_heading);
			right(new_heading);
		}
		else
		{
			heading_aux= -90;
			new_heading= ((int)(heading + heading_aux)) % 360;
			if(new_heading<0) new_heading=(360+new_heading);
			right(new_heading);
		}
	  }
	  else if (command.substring(0,4)=="left")
	  {
	  	if(command.substring(0,5)=="left(")
		{
			heading_aux=get_param(command,5);
			new_heading=((int)(heading + heading_aux )) % 360;
			left((new_heading));
		}
		else
		{
			heading_aux= 90;
			new_heading=((int)(heading + heading_aux )) % 360;
			left(new_heading);
		}
	  }
	  else if (command.substring(0,4)=="stop")
		stopped();
	  else if (command.substring(0,5)=="reset")
		reset();
	  else if (command.substring(0,5)=="state")
		state();
	  
}

/*
 * Set the current command that was popped from queue. The robot
 * will make a move depending of the function called.
 */

void setCommand()
{
    switch(current_state)
    {
      case UP:     up(new_x,new_y);
        break;
      case DOWN:   down(new_x,new_y);
        break;
      case RIGHT:  right(new_heading);
        break;
      case LEFT:   left(new_heading);
        break;
      case STOP:   stopped();
        break;
      default:     stopped();
        break;
    }
}

/*
 * Used to get the parameter of the command given. 
 * @param command, number of character where the command parameter starts
 * @return the command parameter as integer
 */
int get_param(String content, int first)
{
	int i=0,j=0;
	for(i=first; content.substring(i,i+1)!=")"; i++);
	return string_to_int(content.substring(first,i),i-first);
}

/*
 * Convert a number from String to int
 * @param number String, amount of characters (digits of number)
 * @return number as integer
 */
int string_to_int(String s, int digitos)
{
	char num[digitos];
    int size=s.length();
    while(size--) num[size]=s[size];
    return atoi(num);
}

/*
 * Convert a number from String to double
 * @param number String, radix (base)
 * @return number as double
 */
double string_to_double(String s,unsigned short radix)
{
	double n = 0;
	for (unsigned short x=s.length(), y=0; x>0; )
	if(!(s[--x] ^ '.')) 
		n/=pow(10,s.length()-1-x), y+= s.length()-x;
	else
    	n+=( (s[x]-48) * pow(10,s.length()-1-x - y) );
	return n;
}

/*
 * Convert a number from String to float
 * @param number String, amount of characters (digits of number)
 * @return number as float
 */
float string_to_float(String s, int digitos)
{
    char num[digitos];
    int size=s.length();
    while(size--) num[size]=s[size];
    return atof(num);
}
