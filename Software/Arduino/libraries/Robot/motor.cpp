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
 * MotoresTT (GMP36-528 Encoder)-http://www.weiku.com/products/10278880/GMP36_528_Encoder_planetary_gear_motor_with_encoder.html
 * Provide PWM signal to both motors, and combine it to get all robot movements. 
 */
 
#define MOTOR
#include "motor.h"
#include "encoder.h"
#include "control.h"
#include "pin_out.h"
#include "localization.h"

int current_state=STOP;
int PWM_valA = 0, PWM_valB = 0;               // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

/*
 * Set pinout configuration for DC motors and encoders
 */
void motorInit()
{
	pinMode(InA1, OUTPUT);
  	pinMode(InB1, OUTPUT);
  	pinMode(InA2, OUTPUT);
  	pinMode(InB2, OUTPUT);
  	pinMode(encodPinA1, INPUT);
  	pinMode(encodPinB1, INPUT);
}

/*
 * Write a PWM value in InA1 and InB1 motor output pines (h-bridge inputs combination to get 
 * the desired robot motion up). Set maximum values of x,y to checkPosition function. Then update 
 * current robot localization with position() call.
 * @param x,y target values
 */
void up(float max_x, float max_y)
{
  					
  set_max_x(max_x);
  set_max_y(max_y);
  set_max_heading(-500);
  position();
  
  analogWrite(InA1,PWM_valA);
  analogWrite(InB1,PWM_valB);
  analogWrite(InA2,0);
  analogWrite(InB2,0);
  current_state=UP;
}

/*
 * Write a PWM value in InA2 and InB2 motor output pines (h-bridge inputs combination to get 
 * the desired robot motion down). Set maximum values of x,y to checkPosition function. Then update 
 * current robot localization with position() call.
 * @param x,y target values
 */
void down(float max_x,float max_y)
{
  //Serial.println("down");
  set_max_x(max_x);
  set_max_y(max_y);
  set_max_heading(-500);
  position();
  
  analogWrite(InA1,0);
  analogWrite(InB1,0);
  analogWrite(InA2,PWM_valA);
  analogWrite(InB2,PWM_valB);
  current_state=DOWN;
}

/*
 * Write a PWM value in InA1 and InB2 motor output pines (h-bridge inputs combination to get 
 * the desired robot motion left). Set maximum value of heading to checkPosition function. Then update 
 * current robot localization with position() call.
 * @param maximum heading target value
 */
void left(float max_heading)
{
  //Serial.println("left");
  set_max_x( 1000000);
  set_max_y(-1000000);
  set_max_heading(max_heading);
  position();
  
  analogWrite(InA1,PWM_valA);
  analogWrite(InA2,0);
  analogWrite(InB1,0);
  analogWrite(InB2,PWM_valB);
  current_state=LEFT;
}

/*
 * Write a PWM value in InA2 and InB1 motor output pines (h-bridge inputs combination to get 
 * the desired robot motion right). Set maximum value of heading to checkPosition function. Then update 
 * current robot localization with position() call.
 * @param maximum heading target value
 */
void right(float max_heading)
{
  //Serial.println("right");
  set_max_x(-1000000);
  set_max_y( 1000000);
  set_max_heading(max_heading);
  position();
  
  analogWrite(InA1,0);
  analogWrite(InA2,PWM_valA);
  analogWrite(InB1,PWM_valB);
  analogWrite(InB2,0);
  current_state=RIGHT;
}

/*
 * Write zero in all motor output pins (h-bridge inputs combination to get the robot stopped). 
 * Then update current robot localization with position() call.
 */
void stopped()
{
  position();
  //Serial.println("stop");
  analogWrite(InA1,0);
  analogWrite(InA2,0);
  analogWrite(InB1,0);
  analogWrite(InB2,0);
  current_state=STOP;
}

/*
 * Stop the robot and set all position values to zero.
 */
void reset()
{
  countA=0;
  countB=0;
  positionInit();
  stopped();
  current_state=STOP;
}

/*
 * Print robot localization data
 */
void state()
{
	printPositionInfo();
	current_state=STATE;
}

/*
 * Display motor data like speed required, current speed, and encoder counts accumuled.
 * Display frecuency is given by PRINTTIME
 */
void printMotorInfo(float Kp, float Kd, float Ki, float speed_req)
{                                                      
  if((millis()-lastMilliPrint) >= PRINTTIME && current_state!=STOP)
  {
    lastMilliPrint = millis();
    Serial.print("RMPreq:");              	Serial.print(speed_req);
    Serial.print("\tKp:\t");               	Serial.print(Kp);
    Serial.print("\tKd:\t");               	Serial.print(Kd);
    Serial.print("\tKi:\t");               	Serial.print(Ki);
    Serial.print("\tRPMAizq:\t");           Serial.print(speed_actA);
    //Serial.print("\tvel:");            	Serial.print(velA);
    Serial.print("\tPWMA:\t");             	Serial.print(PWM_valA);
    Serial.print("\tEncoder A:\t");       	Serial.print(countA);
    Serial.print("\tRPMBder\t:");           Serial.print(speed_actB);
    //Serial.print("\tvel:");            	Serial.print(velB);
    Serial.print("\tPWMB:\t");             	Serial.print(PWM_valB);
    Serial.print("\tEncoder B:\t");       	Serial.print(countB);
    Serial.println();
  }
}
