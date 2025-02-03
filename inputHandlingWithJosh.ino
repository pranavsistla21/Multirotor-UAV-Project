// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg
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

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
//MPU6050 accelgyro(0x68, &Wire1); // <-- use for AD0 low, but 2nd Wire (TWI/I2C) object

float ax, ay, az;
float gx, gy, gz;

float axC = 0, ayC = 0, azC = 0;
float gxC = 0, gyC = 0, gzC = 0;

float Kp = 2.4, Ki = 0.02, Kd = 1.1; //PID Gain values

// PID variables for the all axeis
float Previous_Error_X = 0, Previous_Error_Y = 0, Previous_Error_Z = 0;
float Integral_X = 0, Integral_Y = 0, Integral_Z = 0;

// The MPU will aim to keep the servos at 90 degress as it's the inital condition
int Set_Point_X = 90;
int Set_Point_Y = 90;
int Set_Point_Z = 90;

float Offset_X = 0, Offset_Y = 0, Offset_Z = 0; // stores intial offset values

#include <Servo.h>
Servo yaw_Servo, pitch_Servo, roll_Servo;


// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
bool blinkState = false;

void setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // turn on LPF to reduce noise
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");



  for (int i = 0; i < 2000; i++) {
    getMPU6050Data(&ax, &ay, &az, &gx, &gy, &gz);
    axC += ax;
    ayC += ay;
    azC += az;
    gxC += gx;
    gyC += gy;
    gzC += gz;
  }
  axC = axC / 2000;
  ayC = ayC / 2000;
  azC = azC / 2000;
  gxC = gxC / 2000;
  gyC = gyC / 2000;
  gzC = gzC / 2000;


  // use the code below to change accel/gyro offset values
  /*
    Serial.println("Updating internal sensor offsets...");
    // -76    -2359    1688    0    0    0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */

  // configure Arduino LED pin for output
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // read raw accel/gyro measurements from device
  getMPU6050Data(&ax, &ay, &az, &gx, &gy, &gz);
  ax -= axC;
  ay -= ayC;
  az -= azC;
  gx -= gxC;
  gy -= gyC;
  gz -= gzC;

  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);

#ifdef OUTPUT_READABLE_ACCELGYRO
  // display tab-separated accel/gyro x/y/z values
  Serial.print("a/g:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.print("\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.println(gz);
#endif

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);


  // Convert accel values to angles and apply offset
    float accelAngle_X = (atan2(ay, az) * 180 / PI) - Offset_X;
    float accelAngle_Y = (atan2(ax, az) * 180 / PI) - Offset_Y;
    float accelAngle_Z = (atan2(ax, ay) * 180 / PI) - Offset_Z;

    //Applying PID controls for the 3-axis
    float output_X = calculatePID(accelAngle_X, Set_Point_X, Previous_Error_X, Integral_X);
    float output_Y = calculatePID(accelAngle_Y, Set_Point_Y, Previous_Error_Y, Integral_Y);
    float output_Z = calculatePID(accelAngle_Z, Set_Point_Z, Previous_Error_Z, Integral_Z);

    // Servo angle limits so that it's within the angle range of the servo motors
    output_X = constrain(output_X, 0, 180);
    output_Y = constrain(output_Y, 0, 180);
    output_Z = constrain(output_Z, 0, 180);

    // function moves servos smoothly
    moveServoSmooth(yaw_Servo, output_X);
    moveServoSmooth(pitch_Servo, output_Y);
    moveServoSmooth(roll_Servo, output_Z);

    // shows the angles in the serial monitor
    Serial.print("Yaw: "); Serial.print(output_X);
    Serial.print(" | Pitch: "); Serial.print(output_Y);
    Serial.print(" | Roll: "); Serial.println(output_Z);

    delay(15); //delay between each consecutive set of readings


}

void getMPU6050Data(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
  int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
  accelgyro.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);

  // Convert raw values to meaningful physical units
  const float ACCEL_SCALE = 16384.0;  // For ±2g
  const float GYRO_SCALE = 131.0;     // For ±250°/s

  *ax = (rawAx / ACCEL_SCALE) * 9.81;  // Convert to m/s²
  *ay = (rawAy / ACCEL_SCALE) * 9.81;
  *az = (rawAz / ACCEL_SCALE) * 9.81;

  *gx = rawGx / GYRO_SCALE;  // Convert to degrees per second (°/s)
  *gy = rawGy / GYRO_SCALE;
  *gz = rawGz / GYRO_SCALE;
}


// PID control code
float calculatePID(float current_Angle, float target_Angle, float &previous_Error, float &Integral) {
    float error = target_Angle - current_Angle;
    Integral += error;
    float derivative = error - previous_Error;

    // PID Output
    float output = target_Angle +(Kp * error) + (Ki * Integral) + (Kd * derivative);

    previous_Error = error; // This stores the previous error to use for the next calculation of error
    return output;

}

// moveServoSmooth function
void moveServoSmooth(Servo &servo, int target_Angle) { // &servo' allows direct manipulation of the servo for smooth and real time movement
    int current_Angle = servo.read(); // reads the current angle of the servo
    while(current_Angle != target_Angle) { // loop always runs as long as current agnle is not equal to taret angle
      if(current_Angle < target_Angle) current_Angle++; // increases angle by 1 degree incerements 
      else current_Angle--; // decreases angle by 1 degree increments

      servo.write(current_Angle); // moves angle to the changed angle
      delay(10); // slows servo movement for smooth movement
      
    }
}