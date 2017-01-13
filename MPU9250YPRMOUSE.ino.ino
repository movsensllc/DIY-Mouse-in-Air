// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//     2012-06-20 - improved FIFO overflow handling and simplified read process
//     2012-06-19 - completely rearranged DMP initialization code and simplification
//     2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//     2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//     2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                - add 3D math helper file to DMP6 example sketch
//                - add Euler output and Yaw/Pitch/Roll output formats
//     2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//     2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//     2012-05-30 - basic DMP initialization working

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
#include <Mouse.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_9Axis_MotionApps41.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw, pitch, roll;

int left_button_pin = 8; // Left button
int right_button_pin = 7; // right button
int leftClickFlag = 0;
const int sensitivity = 30;
float vertZero, horzZero;
float vertValue, horzValue;  // Stores current analog output of each axis

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() 
{
  Wire.begin();                                // join I2C bus (I2Cdev library doesn't do this automatically)
  Serial.begin(115200);                       // initialize serial communication
  while (!Serial);                            // wait for Leonardo enumeration, others continue immediately
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
   if (devStatus == 0) 
   {
      mpu.setDMPEnabled(true);                // turn on the DMP, now that it's ready
      attachInterrupt(0, dmpDataReady, RISING);     // enable Arduino interrupt detection
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;                        // set our DMP Ready flag so the main loop() function knows it's okay to use it
      packetSize = mpu.dmpGetFIFOPacketSize();      // get expected DMP packet size for later comparison
  } 
  else 
  {                                          // ERROR!        1 = initial memory load failed         2 = DMP configuration updates failed        (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
  pinMode(LED_PIN, OUTPUT);                 // configure LED for output
  pinMode(left_button_pin, INPUT);
  pinMode(right_button_pin, INPUT);
  yaw = 0.0;
  pitch = 0.0;
  roll = 0.0;
  vertZero = 0;
  horzZero = 0;
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

    if (!dmpReady) return;                                                    // if programming failed, don't try to do anything
    mpuInterrupt = true;
    fifoCount = mpu.getFIFOCount();                                           // get current FIFO count
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)                           // check for overflow (this should never happen unless our code is too inefficient)
    {
        mpu.resetFIFO();                                                      // reset so we can continue cleanly
        Serial.println(F("FIFO overflow!"));
    } 
    else if (mpuIntStatus & 0x01)                                             // otherwise, check for DMP data ready interrupt (this should happen frequently)
    {    
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();        // wait for correct available data length, should be a VERY short wait
        mpu.getFIFOBytes(fifoBuffer, packetSize);                             // read a packet from FIFO
        fifoCount -= packetSize;                                              // track FIFO count here in case there is > 1 packet available
        #ifdef OUTPUT_READABLE_YAWPITCHROLL                                               // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw = ypr[1] /PI * 180;
            pitch = ypr[2] /PI * 180;
            roll = ypr[0] /PI * 180;
            Serial.print("ypr\t");
            Serial.print(yaw);
            Serial.print("\t");
            Serial.print(pitch);
            Serial.print("\t");
            Serial.println(roll);
        #endif
        blinkState = !blinkState;                                             // blink LED to indicate activity
        vertValue = yaw - vertZero;
        horzValue = roll - horzZero;
        vertZero = yaw;
        horzZero = roll;   
        if (vertValue != 0)
          Mouse.move(0, vertValue * sensitivity, 0);                                      // move mouse on y axis
        if (horzValue != 0)
          Mouse.move(horzValue * sensitivity, 0, 0);                                      // move mouse on x axis

        if ((digitalRead(left_button_pin))&&(!leftClickFlag))
        {
          leftClickFlag = 1;
          Mouse.press(MOUSE_LEFT);
        }
        else if ((digitalRead(left_button_pin))&&(leftClickFlag))
        {
          leftClickFlag = 0;
          Mouse.release(MOUSE_LEFT);
        }
        if (digitalRead(right_button_pin))
        {
          Mouse.click(MOUSE_RIGHT);
        }
    }
}
