#include <Wire.h>
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <MPU6050.h>

#define SD_CS 4  // Select pin for SD card module

RTC_DS3231 rtc;
MPU6050 mpu;
File dataFile;

// ----- Servo Objects -----
Servo servoYaw, servoPitch, servoRoll;

// ----- PID Variables -----
double inputYaw, outputYaw, setpointYaw = 0;
double inputPitch, outputPitch, setpointPitch = 0;
double inputRoll, outputRoll, setpointRoll = 0;

// ----- Sensor Low-Pass Filter Constants -----
const double LPF_ALPHA = 0.75;
const double LPF_ALPHA_ROLL = 0.65;

// ----- Filtered Sensor Values -----
double filteredYaw = 0, filteredPitch = 0, filteredRoll = 0;

// ----- PID Tuning Parameters -----
PID pidYaw(&inputYaw, &outputYaw, &setpointYaw, 1.5, 0.0, 0.5, DIRECT);
PID pidPitch(&inputPitch, &outputPitch, &setpointPitch, 2.5, 0.005, 0.001, DIRECT);
PID pidRoll(&inputRoll, &outputRoll, &setpointRoll, 1.5, 0.005, 0.002, DIRECT);

// ----- MPU6050 DMP Variables -----
uint8_t mpuIntStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];  // [yaw, pitch, roll]

// ----- Offsets for Zeroing -----
double yawOffset = 0, pitchOffset = 0, rollOffset = 0;

// ----- Servo Limits -----
#define SERVO_MIN 0
#define SERVO_MAX 180

// ----- Incremental Servo Movement Variables -----
int currentServoYaw   = 90;  
int currentServoPitch = 90;
int currentServoRoll  = 90;
const int MAX_STEP = 2;

// ----- Dynamic Yaw Drift Correction Variables -----
float dynamicDriftOffset = 0.0;
const float DRIFT_CORRECTION_FACTOR = 0.02;
float lastFilteredYaw = 0;
float yawHold = 0;

// ----- Additional Output Filtering Variables -----
const double LPF_OUTPUT_ALPHA = .98;
static double filteredOutputYaw = 0, filteredOutputPitch = 0, filteredOutputRoll = 0;

int updateServoPos(int currentPos, int targetPos, int step) {
  if (abs(targetPos - currentPos) <= step)
    return targetPos;
  else if (targetPos > currentPos)
    return currentPos + step;
  else
    return currentPos - step;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card initialization failed!");
    while (1);
  }
  Serial.println("SD Card initialized.");

  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  Serial.println("Enabling DMP...");
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
  Serial.println("MPU6050 Ready");

  delay(1000);

  const int NUM_SAMPLES = 50;
  double sumYaw = 0, sumPitch = 0, sumRoll = 0;
  int validSamples = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    if ((mpuIntStatus & 0x10) || mpu.getFIFOCount() >= 1024) {
      mpu.resetFIFO();
      Serial.println("FIFO Overflow during calibration! Resetting...");
      continue;
    }
    while (mpu.getFIFOCount() < packetSize) {
      delay(5);
    }
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    sumYaw   += ypr[0] * 180.0 / M_PI;
    sumPitch += ypr[1] * 180.0 / M_PI;
    sumRoll  += ypr[2] * 180.0 / M_PI;
    validSamples++;
    delay(10);
  }
  if (validSamples > 0) {
    yawOffset   = sumYaw / validSamples;
    pitchOffset = sumPitch / validSamples;
    rollOffset  = sumRoll / validSamples;
  }
  Serial.print("Offsets -> Yaw: "); Serial.print(yawOffset);
  Serial.print(", Pitch: "); Serial.print(pitchOffset);
  Serial.print(", Roll: "); Serial.println(rollOffset);

  servoYaw.attach(11);
  servoPitch.attach(9);
  servoRoll.attach(10);

  pidYaw.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);
  pidRoll.SetMode(AUTOMATIC);

  pidYaw.SetOutputLimits(-60, 60);
  pidPitch.SetOutputLimits(-60, 60);
  pidRoll.SetOutputLimits(-60, 60);

  servoYaw.write(currentServoYaw);
  servoPitch.write(currentServoPitch);
  servoRoll.write(currentServoRoll);

  yawHold = 0;
}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  DateTime now = rtc.now();

  dataFile = SD.open("log.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(now.year()); dataFile.print("/");
    dataFile.print(now.month()); dataFile.print("/");
    dataFile.print(now.day()); dataFile.print(" ");
    dataFile.print(now.hour()); dataFile.print(":");
    dataFile.print(now.minute()); dataFile.print(":");
    dataFile.print(now.second()); dataFile.print(", ");
    dataFile.print(ax); dataFile.print(", ");
    dataFile.print(ay); dataFile.print(", ");
    dataFile.print(az); dataFile.print(", ");
    dataFile.print(filteredYaw); dataFile.print(", ");
    dataFile.print(filteredPitch); dataFile.print(", ");
    dataFile.print(filteredRoll); dataFile.print(", ");
    dataFile.print(currentServoYaw); dataFile.print(", ");
    dataFile.print(currentServoPitch); dataFile.print(", ");
    dataFile.print(currentServoRoll); dataFile.print(", ");
    dataFile.print(outputYaw); dataFile.print(", ");
    dataFile.print(outputPitch); dataFile.print(", ");
    dataFile.println(outputRoll);
    dataFile.close();
  }

  if ((mpuIntStatus & 0x10) || mpu.getFIFOCount() >= 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO Overflow! Resetting...");
    return;
  }

  if (mpu.getFIFOCount() >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    double rawYaw   = (ypr[0] * 180.0 / M_PI) - yawOffset;
    double rawPitch = (ypr[1] * 180.0 / M_PI) - pitchOffset;
    double rawRoll  = (ypr[2] * 180.0 / M_PI) - rollOffset;

    filteredYaw   = LPF_ALPHA * rawYaw   + (1 - LPF_ALPHA) * filteredYaw;
    filteredPitch = LPF_ALPHA * rawPitch + (1 - LPF_ALPHA) * filteredPitch;
    filteredRoll  = LPF_ALPHA_ROLL * rawRoll + (1 - LPF_ALPHA_ROLL) * filteredRoll;

    float yawRate = abs(filteredYaw - lastFilteredYaw);
    lastFilteredYaw = filteredYaw;

    if (yawRate < 1.0) {
      dynamicDriftOffset = (1 - DRIFT_CORRECTION_FACTOR) * dynamicDriftOffset +
                           DRIFT_CORRECTION_FACTOR * filteredYaw;
    }
    float correctedYaw = filteredYaw - dynamicDriftOffset;

    if (yawRate > 2.0) {
      int targetYawPos = constrain(map(correctedYaw, -60, 60, SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX);
      currentServoYaw = updateServoPos(currentServoYaw, targetYawPos, MAX_STEP);
    } else {
      yawHold = correctedYaw;
      inputYaw = correctedYaw - yawHold;
      pidYaw.Compute();
      filteredOutputYaw = LPF_OUTPUT_ALPHA * outputYaw + (1 - LPF_OUTPUT_ALPHA) * filteredOutputYaw;
      const double DEAD_BAND = 14.0;
      if (abs(filteredOutputYaw) < DEAD_BAND)
        filteredOutputYaw = 0;
      int targetYawPos = constrain(map(filteredOutputYaw, -60, 60, SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX);
      currentServoYaw = updateServoPos(currentServoYaw, targetYawPos, MAX_STEP);
    }

    inputPitch = filteredPitch;
    inputRoll = filteredRoll;
    pidPitch.Compute();
    pidRoll.Compute();
    filteredOutputPitch = LPF_OUTPUT_ALPHA * outputPitch + (1 - LPF_OUTPUT_ALPHA) * filteredOutputPitch;
    filteredOutputRoll  = LPF_OUTPUT_ALPHA * outputRoll  + (1 - LPF_OUTPUT_ALPHA) * filteredOutputRoll;
    if (abs(filteredOutputPitch) < 8.0) filteredOutputPitch = 0;
    if (abs(filteredOutputRoll) < 8.0)  filteredOutputRoll  = 0;
    int targetPitchPos = constrain(map(filteredOutputPitch, -60, 60, SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX);
    int targetRollPos  = constrain(map(filteredOutputRoll,  -60, 60, SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX);

    currentServoPitch = updateServoPos(currentServoPitch, targetPitchPos, MAX_STEP);
    currentServoRoll  = updateServoPos(currentServoRoll,  targetRollPos,  MAX_STEP);

    servoYaw.write(currentServoYaw);
    servoPitch.write(currentServoPitch);
    servoRoll.write(currentServoRoll);

    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 200) {
      lastUpdate = millis();
      Serial.print("Raw -> Yaw: "); Serial.print(rawYaw);
      Serial.print(" | Pitch: "); Serial.print(rawPitch);
      Serial.print(" | Roll: "); Serial.print(rawRoll);
      Serial.print(" || Filtered -> Yaw: "); Serial.print(filteredYaw);
      Serial.print(" || Corrected Yaw: "); Serial.print(correctedYaw);
      Serial.print(" || YawRate: "); Serial.print(yawRate);
      Serial.print(" || Servo -> Yaw: "); Serial.println(currentServoYaw);
    }
  }
  delay(2);
}
