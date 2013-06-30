// This code was only restructured by Agustin Caverzasi <agucaverzasi@gmial.com> and 
// Fernando Saravia <fersarr@gmail.com> to get a Gyroscope class and call some functions to get his data

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/


#ifndef _GYROSCOPE_H_
#define _GYROSCOPE_H_

#include "Arduino.h"

//#include "I2Cdev.h"
//#include "Wire.h"
//#include "stdint.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define GET_QUATERNION
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define GET_EULER
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define GET_YAWPITCHROLL
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define GET_REALACCEL
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define GET_WORLDACCEL
//#define OUTPUT_READABLE_WORLDACCEL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

#ifdef GYRO
	bool blinkState;
	bool dmpReady;		    // MPU control/status vars. set true if DMP init was successful
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	float yaw;
#endif			

class Gyroscope
{
 	private:		 
	 	   	// MPU control/status vars
		  	bool blinkState;
			bool dmpReady;		    // MPU control/status vars. set true if DMP init was successful
			//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
			//uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
			//uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
			//uint16_t fifoCount;     // count of all bytes currently in FIFO
			//uint8_t fifoBuffer[64]; // FIFO storage buffer
			
			// orientation/motion vars	
			float euler[3];         // [psi, theta, phi]    Euler angle container
			
			//print functions
			void printQuaternion();
        	void printEuler();
            void printYawPitchRoll();
            void printRealAccel();
          	void printWorldAccel(); 
          	
          	void setLastYaw(float ypr[3]);
 	public: 
		 	Gyroscope();    
			void mpuInit();
			void gyroGetData();
			void joinI2Cbus(); 
			float getLastYaw();
 	
};

//extern Gyroscope gyro; //para hacer la clase estatica

#endif


