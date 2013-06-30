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
 
#ifndef _LOCALIZATION_H_
#define _LOCALIZATION_H_

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Gyroscope.h>
#include <QueueList.h>

extern Gyroscope g;
extern float headingBase, heading;
extern volatile double x,y;
extern float max_heading, max_x, max_y;
extern volatile double pulsosAntA, pulsosAntB;
extern int cuenta;

void gyroInit();
void position();
void positionInit();
void printPositionInfo();
double getX();
double getY();
double getHeading();
double getHeadingRad();
void set_max_heading(float);
void set_max_x(float);
void set_max_y(float);
void checkPosition(QueueList<String> &q);
bool isInBounds(float value, float low, float high);
void nextCommand(QueueList<String> &q);

#endif
