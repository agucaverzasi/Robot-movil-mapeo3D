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
 
#ifndef _CONTROL_H_
#define _CONTROL_H_

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <QueueList.h>

#define LOOPTIME        10                      // PID loop time
#define PRINTTIME       10                      // print time

extern double accum_errorA, accum_errorB;		
extern double error_promA, error_promB;
extern double velA, velB;
extern int counterA, counterB;
extern unsigned long lastMilli;                  // loop timing
extern unsigned long lastMilliPrint;             // loop timing

int updatePidA(int command, double targetValue, double currentValue, float Kp, float Kd, float Ki);
int updatePidB(int command, double targetValue, double currentValue, float Kp, float Kd, float Ki);
void run(float speed_req, float Kp, float Kd, float Ki, bool printMotor, bool printPosition,QueueList<String> &q);
void setCommand();
int getCommand(QueueList<String> &q);


#endif
