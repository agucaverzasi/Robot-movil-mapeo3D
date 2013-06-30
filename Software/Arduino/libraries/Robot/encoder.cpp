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
 * Implement some functions for encoders odometry, and figure out both DC motors speed.
 */ 
 
#define ENCODER
#include "encoder.h"
#include "control.h"
#include "pin_out.h"

volatile long pulsosA = 0, pulsosB = 0; 		 // encoder pulses
volatile long countA = 0, countB = 0;            // encoder pulses
double speed_actA = 0, speed_actB = 0;           // speed (actual value)

/*
 * Enable external interrupts to read encoder signal
 */
void encoderInit()
{
	digitalWrite(encodPinA1, HIGH);                     // turn on pullup resistor
	digitalWrite(encodPinB1, HIGH);
	attachInterrupt(4, rencoderB, FALLING); 			// int4 = pin 19
	attachInterrupt(5, rencoderA, FALLING); 			// int5 = pin 18
}

/*
 * Read pulses from left encoder.
 * This function is called each time an external interrup event ocurr.
 * It increment the count to save cycles, reading the port directly.
 */
void rencoderA()  
{
     if (PIN18_HIGH)
          countA++;
     else
          countA--;
}

/*
 * Read pulses from right encoder.
 * This function is called each time an external interrup event ocurr.
 * It increment the count to save cycles, reading the port directly.
 */
void rencoderB()  
{
        if (PIN19_HIGH)
          countB++;
        else
          countB--;
} 

/*
 * Calculate both DC motors speed in [rpm].
 */
void getEncoderCount()
{                    
    static long countAntA = 0;
    static long countAntB = 0;                                                   

    //Left motor. Pulses per revolution (4234.37) was obtained under experiment.
    if(countA!=countAntA)
      	speed_actA = ((countA - countAntA)*(60*(1000/LOOPTIME)))/4234.378; 
    countAntA = countA;		//Last count
    //Right motor. Pulses per revolution (4250.57) was obtained under experiment.
    if(countB!=countAntB)
    	speed_actB = ((countB - countAntB)*(60*(1000/LOOPTIME)))/4250.575; 
    countAntB = countB;		//Last count
}


