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
 *
 */

#include <control.h>
#include <motor.h>
#include <pin_out.h>
#include <encoder.h>
#include <commands.h>
#include <localization.h>
#include <Gyroscope.h>
#include <QueueList.h>

QueueList<String> queue;

void setup() 
{
  /*
   * Inicializa la conexion entre Arduino y la PC onboard a un baud rate de 115200.
   * (al abrir el serial monitor con shift+m recordar setear en la ventana inferior el baud rate a 115200 y new line). 
   * -motorInit()->setea los pines de Arduino conectados a motores y encoders
   * -encoderInit()->configura interrupciones externas en pines de encoders   
   * -gyroInit()->inicializa el giroscopo que tarda un tiempo en estabilizarse, cuando
   * finaliza avisa en el serial monitor
   * -positionInit()->inicia la localizacion a bajo nivel del robot
   */
  Serial.begin(115200);  
  motorInit();
  encoderInit();
  gyroInit();
  positionInit();
  Serial.println("Done");
}

void loop()
{
  
  /*
   * funcion run() se define en Robot/control.h
   * argumentos:
   * vel_req, (velocidad del robot)
   * kp, (controlador PID)
   * kd, (controlador PID)
   * ki, (controlador PID)
   * print_pid, (imprimir datos del PID para debugging)
   * print_localizacion, (imprime datos de localizacion para debugging)
   * queue (buffer donde se almacenan los comandos que llegan desde la PC)
   */  
  run(10,2.5,3.5,0.09,false,true,queue);
}
