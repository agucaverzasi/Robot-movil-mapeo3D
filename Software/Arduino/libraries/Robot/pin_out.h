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
 

#ifndef _PIN_OUT_H_
#define _PIN_OUT_H_

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#else
	#include "WProgram.h"
#endif

//Motors pin out (Arduino PWM outputs)
#define InA1            9                       // INA motor pin
#define InA2            10                      // INA motor pin
#define InB1            11                      // INB motor pin
#define InB2            12                      // INB motor pin

//Encoder (Arduino external interrupt inputs)
#define encodPinA1      18                      // encoder A pin
#define encodPinB1      19                      // encoder B pin
#define PIN18_HIGH  PORTD3               		//(PORTD & 0b00000100)
#define PIN19_HIGH  PORTD2              		//(PORTD & 0b00000010)

#endif
