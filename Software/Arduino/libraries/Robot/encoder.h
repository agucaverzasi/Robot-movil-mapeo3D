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
 
#ifndef _ENCODER_H_
#define _ENCODER_H_

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#else
	#include "WProgram.h"
#endif

extern volatile long pulsosA, pulsosB; 
extern volatile long countA, countB; 
extern double speed_actA, speed_actB;           // speed (actual value)

void encoderInit();
void rencoderA();
void rencoderB();
void getEncoderCount();

#endif
