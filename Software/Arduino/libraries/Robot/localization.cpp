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
 * Uses the encoder odometry and gyroscope data to localize the robot and update in each new motion.
 */

#define LOCALIZATION
#include "localization.h"
#include "Gyroscope.h"
#include "motor.h"
#include "encoder.h"
#include "control.h"
#include "commands.h"


//wheels diameter = 10cm -> the distance advanced per pulse is (10cm*pi/pulses_per_revolution)
#define cm_pulsoA 0.007419// diameter * pi / pulses per revolution -> 10*pi/4234.378
#define cm_pulsoB 0.007390// diameter * pi / pulses per revolution -> 10*pi/4250.575

volatile double pulsosAntA=0, pulsosAntB=0;
volatile double x=0,y=0;
float headingBase=0, heading=0;
int cuenta=0;
Gyroscope g;
float max_heading=0, max_x=0, max_y=0;

/*
 * Initialize Gryroscope library. It takes a few minutes to get stable values. 
 * Print a message when initialization is done.
 */
void gyroInit()
{
	Serial.println("Initializing Gyroscope, wait please...");
	g.mpuInit();
	g.joinI2Cbus();
  
	while(cuenta<10000)
	{
		g.gyroGetData();
		heading=g.getLastYaw();
		cuenta++;
	}
	Serial.println("Initializing Gyroscope, done");
}

/*
 * Set initial robot position at (x,y)=(0,0) and get the initial angle value.
 */
void positionInit()
{
	heading=0;
	x=0;
	y=0;
	g.gyroGetData();				// ask for yaw,pitch,roll gyroscope data
  	headingBase=g.getLastYaw();		// get yaw value (robot heading)
  	Serial.print("Heading base: ");  Serial.println(headingBase);
  	
  	pulsosAntA=0;	// encoder pulses counter
  	pulsosAntB=0;	// encoder pulses counter
}

/*
 * Is called from each robot motion function in Robot/motor.cpp
 * Calculates the advanced distance from encoder pulses difference multiplied by the 
 * distance that every encoder pulse represent. 
 * Then figure out x,y coordinates from the product of this distance and sin/cos
 * of the current robot heading in radians.
 */
void position()
{
	volatile double dist=( (countA-pulsosAntA) * cm_pulsoA + (countB-pulsosAntB) * cm_pulsoB ) / 2;
	pulsosAntA=countA;
	pulsosAntB=countB;
	
	g.gyroGetData();
	heading = g.getLastYaw() - headingBase;
	if( heading < 0 ) heading=heading+360;   
    
	float headingRad=heading*2*PI/360;
    
    volatile double y1=dist*sin(headingRad);
    volatile double x1=dist*cos(headingRad);
  
   	switch(current_state)
   	{
   		case UP:
   				x+=x1;
   				y+=y1;
   				break;
   		case DOWN:
   				x-=x1;
   				y-=y1;
   				break;
   		default: 
   				break;
   	}
}

/*
 * Display robot localization data (heading,x,y)
 * Display frecuency is given by PRINTTIME
 */
void printPositionInfo()
{                                                      // display data
  if((millis()-lastMilliPrint) >= PRINTTIME && current_state!=STOP)
  {
    lastMilliPrint = millis();
    Serial.print("Robot: [ heading:\t");	Serial.print(heading);
    Serial.print("\tx:\t");					Serial.print(x);
    Serial.print("\ty:\t");					Serial.print(y);
    Serial.println("\t]");
  }
}

double getX(){ return x; }

double getY(){ return y; }

double getHeading(){ return heading; }

double getHeadingRad(){ return getHeading()*2*PI/360; }

void set_max_heading(float value){ max_heading=value; }

void set_max_x(float value){ max_x=value; }

void set_max_y(float value){ max_y=value; }

/*
 * Check if a value is in between a given range (low,high)
 * @return true if value is in range, otherwise false
 */
bool isInBounds(float value, float low, float high) 
{
    return !(value < low) && !(high < value);
}

/*
 * Check if the robot is "near" to his target position in x,y or heading
 * to set the next command movement.
 */
int count=0;
void checkPosition(QueueList<String> &q)
{
	 if(current_state==STOP)
	 	nextCommand(q);
	 else
	 {
		 if( isInBounds(heading, max_heading, max_heading + 1) )	//rotation motion
		 {
		 	printPositionInfo();
		 	nextCommand(q);
		 	//stopped();
		 }
		 else	//lineal motion
		 { 
		 	float v=(((max_y-getY())*(max_y-getY()))+((max_x-getX()))*((max_x-getX()))); //dx*dx+dy*dy
			//Uncomment this lines for localization debugg
			/*
			count++;
			if(count>20)
			{
				Serial.print("              ");Serial.println(v); 
				Serial.println("x");	Serial.println(max_x); Serial.println(getX());
				Serial.println("y");  Serial.println(max_y); Serial.println(getY());
				Serial.println(max_y-getY()); Serial.println(max_x-getX());
			}
			*/
			
		   /* Whether dx*dx+dy*dy <= delta the robot reaches the localization target.
			* Now is set to move forward or backward 10cm for each up or down command. But if you
			* can especify a bigger amount to this movements, the recommended delta values are:
			* Go forward/backward 10cm -> delta = 0.09
			* Go forward/backward 20cm -> delta = 0.25
            * Go forward/backward 50cm -> delta = 2
            * Go forward/backward 100cm-> delta = 21
			* (for more information see Localizacion setion in 
			* https://github.com/agucaverzasi/Robot-movil-mapeo3D/blob/master/Informe.pdf).
			*/
			if(v<=0.09) 
		 	{
		 		printPositionInfo();
		 		nextCommand(q);
		 		//stopped();
		 	}
		 }
	 }
}
