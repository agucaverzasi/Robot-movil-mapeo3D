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
 * Implementation of robot control. As the robot is differential wheeled, the aim is to get the 
 * robot movement in a stright line for lineal movements, and make a rotation about the central 
 * point of the axis, for rotational movements. 
 * Since the direction of the robot is dependent on the rate and direction of rotation of the two 
 * driven wheels, these quantities should be sensed and controlled precisely.
 *
 * Credits: 
 *
 *  User "kas" (http://forum.arduino.cc/index.php/topic,8652.0.html) for "updatePID()" function
 *
 */ 

#define CONTROL
#include "control.h"
#include "motor.h"
#include "encoder.h"
#include "commands.h"
#include "localization.h"


double accum_errorA=0, accum_errorB=0;		  //error accumulated along time
double error_promA=0, error_promB=0;		  //average error
int counterA=0, counterB=0;					  //increments in each updatePID function call

unsigned long lastMilli = 0;                  // loop timing
unsigned long lastMilliPrint = 0;             // loop timing

/* 
 * PID controller correction function
 * given a motor desired speed [rpm], return the new duty cycle value for PWM motor input.
 * Is called from run()
 * @param current duty cycle value of left PWM motor, left motor speed required [rpm],
 * current motor speed[rpm], PID controller parameters (Kp,Kd,Ki)
 */
int updatePidA(int command, double targetValue, double currentValue, float Kp, float Kd, float Ki)
{
    double pidTerm = 0;                                                            
    double error=0;
    static double last_error=0;
          error = abs(targetValue) - abs(currentValue);									//figure out current error
      	  pidTerm = (Kp * error) + (Kd * (error - last_error)) + (Ki * accum_errorA);	//PID function
     	  last_error = error;
          accum_errorA+=error;															//accumulated error for Ki term
          counterA++;
          error_promA=accum_errorA/counterA;
      	  return constrain(command + int(pidTerm), 0, 255);								//return a value between 0-255 (PWM motor input)
}

/* 
 * PID controller correction function
 * given a motor desired speed [rpm], return the new duty cycle value for PWM motor input.
 * Is called from run()
 * @param current duty cycle value of right PWM motor, right motor speed required [rpm],
 * current motor speed[rpm], PID controller parameters (Kp,Kd,Ki)
 * @return new duty cycle value for PWM right motor input
 */
int updatePidB(int command, double targetValue, double currentValue, float Kp, float Kd, float Ki)
{
    double pidTerm = 0;                                                           
    double error=0;
    static double last_error=0;
          error = abs(targetValue) - abs(currentValue);									//figure out current error
     	  pidTerm = (Kp * error) + (Kd * (error - last_error)) + (Ki * accum_errorB);	//PID function
     	  last_error = error;
          accum_errorB+=error;															//accumulated error for Ki term
          counterB++;
          error_promB=accum_errorB/counterB;
          /* DC right motor (seen in front) is faster than left, so this function return a value between 0-247 to 
           * constrain the initial movement velocity (transitional period), then when they reach his stable state, 
           * thay are auto-fited to the desired velocity
           * The maximum constrain value depends of the battery voltage level (to work with maximum velocity).
           */
          return constrain(command + int(pidTerm), 0, 247);								
}

/* 
 * Update current robot position, get next command from buffer, 
 * check the actual robot position, set new command to carry out new robot movement,
 * finally update the PWM values of both motors calling each PID update function respectively.
 * This function is called all the time (periodically) from loop() (Sketchbook/robot_arduino.ino)
 * @param robot speed required, PID controller parameters (Kp,Kd,Ki), print motor information, 
 * print robot localization with encoders information, buffer with commands coming from ROS 
 * @return new duty cycle value for PWM right motor input
 */
void run(float speed_req, float Kp, float Kd,float Ki, bool printMotor, bool printPosition, QueueList<String> &q)
{

    if((millis()-lastMilli) >= LOOPTIME) // enter tmed loop
    {
      lastMilli = millis();
	  position();
	  getCommand(q);  		
	  checkPosition(q);
      setCommand();

      getEncoderCount();

      PWM_valA = updatePidA(PWM_valA, speed_req, speed_actA, Kp, Kd, Ki);  //left motor
      PWM_valB = updatePidB(PWM_valB, speed_req, speed_actB, Kp, Kd, Ki);  //right motor (faster)
    }
    if(printMotor)	
    	printMotorInfo(Kp,Kd,Ki,speed_req);  
	if(printPosition)	
   		printPositionInfo(); 
}


