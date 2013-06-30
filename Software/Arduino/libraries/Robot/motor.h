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
 
#ifndef _MOTOR_H_
#define _MOTOR_H_

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#else
	#include "WProgram.h"
#endif
  
#define InA1            9                       // INA motor pin
#define InA2            10                      // INA motor pin
#define InB1            11                      // INB motor pin
#define InB2            12                      // INB motor pin  

extern int PWM_valA, PWM_valB;               // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

enum command{
    UP=0,
    DOWN=1,
    LEFT=2,
    RIGHT=3,
    STOP=4,
    RESET=5,
    STATE=6
};

extern int current_state;

void motorInit();
void up(float,float);
void down(float,float);
void right(float max_heading);
void left(float max_heading);
void stopped();
void reset();
void state();
void printMotorInfo(float Kp, float Kd, float Ki, float speed_req);

#endif
