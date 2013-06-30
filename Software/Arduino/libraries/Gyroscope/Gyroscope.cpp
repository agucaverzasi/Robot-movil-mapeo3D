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

#define GYRO
#include "Gyroscope.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector  

volatile bool mpuInterrupt;

void dmpDataReady() {
    mpuInterrupt = true;
	//Serial.print("yaaaw" );
	//Serial.println(yaw);
}

Gyroscope::Gyroscope() {
	blinkState = false;
	dmpReady = false;  // MPU control/status vars. set true if DMP init was successful
	mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
}

void Gyroscope::mpuInit()
{

    // initialize serial communication
    //Serial.begin(9600);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(0, &(dmpDataReady), RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("Digital Motion Processing ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else 
    {
        Serial.print(F("Digital Motion Processing Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

void Gyroscope::gyroGetData()
{
	//Serial.println(" En gyroData ");
  	// if programming failed, don't try to do anything
    if (!dmpReady){
		//Serial.println("DMP no listo");
		 return;
	}
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    
    // get current FIFO count and check for FIFO overflow (a big amount of interrupts)
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if (( (mpuIntStatus & 0x10) || fifoCount == 1024)){
        mpu.resetFIFO();	// reset so we can continue cleanly
		//Serial.println("FIFO LLENA o intStatus SALE TEMPRANO");
    }
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    else 
    	if (mpuIntStatus & 0x02) 
    	{
		    // wait for correct available data length, should be a VERY short wait
		    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		    // read a packet from FIFO
		    mpu.getFIFOBytes(fifoBuffer, packetSize);
		    
		    // track FIFO count here in case there is > 1 packet available
		    // (this lets us immediately read more without waiting for an interrupt)
		    fifoCount -= packetSize;
				
			#ifdef GET_YAWPITCHROLL
				//Serial.println("Entro a parte de yaw");
				mpu.dmpGetQuaternion(&q, fifoBuffer);
				mpu.dmpGetGravity(&gravity, &q);
				mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
				setLastYaw(ypr);							//save the last degrees read from FIFO   
			#endif		
			
			#ifdef OUTPUT_READABLE_YAWPITCHROLL
		    	printYawPitchRoll();
		    #endif
		}
	else{
		//Serial.println("INT no lista");
	}
}

void Gyroscope::printWorldAccel()
{
	// display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
}

void Gyroscope::printRealAccel()
{
	// display real acceleration, adjusted to remove gravity
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
}

void Gyroscope::printYawPitchRoll()
{
	// display Euler angles in degrees
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);
}

void Gyroscope::printEuler()
{
    // display Euler angles in degrees
    Serial.print("euler\t");
    Serial.print(euler[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180/M_PI);
}

void Gyroscope::printQuaternion()
{
	// display quaternion values in easy matrix form: w x y z
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
}

float Gyroscope::getLastYaw()
{
	return yaw;
}

void Gyroscope::setLastYaw(float ypr[3])
{
	yaw=ypr[0] * 180/M_PI;
}

void Gyroscope::Gyroscope::joinI2Cbus()
{
	// join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
	
}
