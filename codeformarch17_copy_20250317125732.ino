#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <RTClib.h>
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>

#define SD_CS 4  // Chip Select pin for SD card module
#define SERVO_MIN 0
#define SERVO_MAX 180

// ----- Global Objects -----
RTC_DS3231 rtc;
MPU6050 mpu;
File dataFile;

// ----- DMP Variables -----
uint8_t mpuIntStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
VectorInt16 aa; // For raw acceleration data from DMP

// ----- Servo and PID Setup -----
Servo servoYaw, servoPitch, servoRoll;
double inputYaw, outputYaw, setpointYaw = 0;
double inputPitch, outputPitch, setpointPitch = 0;
double inputRoll, outputRoll, setpointRoll = 0;

// ----- Low-Pass Filter Variables -----
double filteredYaw = 0, filteredPitch = 0, filteredRoll = 0;
const double LPF_ALPHA = 0.2;  // Smoothing factor (0 < LPF_ALPHA < 1)

// ----- PID Tuning Parameters -----
double Kp = 2.0, Ki = 0.01, Kd = 0.05;
PID pidYaw(&inputYaw, &outputYaw, &setpointYaw, Kp, Ki, Kd, DIRECT);
PID pidPitch(&inputPitch, &outputPitch, &setpointPitch, Kp, Ki, Kd, DIRECT);
PID pidRoll(&inputRoll, &outputRoll, &setpointRoll, Kp, Ki, Kd, DIRECT);

// ----- Offsets for Zeroing Initial Position -----
double yawOffset = 0, pitchOffset = 0, rollOffset = 0;

// ----- Timing Variables -----
unsigned long lastLogTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize RTC
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  // Initialize SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card initialization failed!");
    while (1);
  }
  Serial.println("SD Card initialized.");

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  // Initialize DMP for orientation processing
  Serial.println("Initializing DMP...");
  int dmpStatus = mpu.dmpInitialize();
  if (dmpStatus != 0) {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(dmpStatus);
    Serial.println(")");
    while (1);
  }
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
  Serial.println("MPU6050 DMP Ready");

  // Allow sensor readings to stabilize
  delay(1000);

  // Calibrate offsets using multiple samples
  const int NUM_SAMPLES = 50;
  double sumYaw = 0, sumPitch = 0, sumRoll = 0;
  int validSamples = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    // Check for FIFO overflow during calibration
    if ((mpu.getIntStatus() & 0x10) || mpu.getFIFOCount() >= 1024) {
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

  // Attach servos to their pins
  servoYaw.attach(11);
  servoPitch.attach(9);
  servoRoll.attach(10);

  // Initialize PID controllers
  pidYaw.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);
  pidRoll.SetMode(AUTOMATIC);
  pidYaw.SetOutputLimits(-60, 60);
  pidPitch.SetOutputLimits(-60, 60);
  pidRoll.SetOutputLimits(-60, 60);
}

void loop() {
  // Check for FIFO overflow and reset if needed
  if ((mpu.getIntStatus() & 0x10) || mpu.getFIFOCount() >= 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO Overflow! Resetting...");
    return;
  }

  // Process a new DMP packet if available
  if (mpu.getFIFOCount() >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // Retrieve orientation data from DMP packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Retrieve raw accelerometer data from the same packet
    mpu.dmpGetAccel(&aa, fifoBuffer);

    // Calculate adjusted raw angles (in degrees)
    double rawYaw   = (ypr[0] * 180.0 / M_PI) - yawOffset;
    double rawPitch = (ypr[1] * 180.0 / M_PI) - pitchOffset;
    double rawRoll  = (ypr[2] * 180.0 / M_PI) - rollOffset;

    // Apply low-pass filtering to smooth the values
    filteredYaw   = LPF_ALPHA * rawYaw   + (1 - LPF_ALPHA) * filteredYaw;
    filteredPitch = LPF_ALPHA * rawPitch + (1 - LPF_ALPHA) * filteredPitch;
    filteredRoll  = LPF_ALPHA * rawRoll  + (1 - LPF_ALPHA) * filteredRoll;

    // Use the filtered values as PID inputs
    inputYaw   = filteredYaw;
    inputPitch = filteredPitch;
    inputRoll  = filteredRoll;

    // Compute PID corrections for each axis
    pidYaw.Compute();
    pidPitch.Compute();
    pidRoll.Compute();

    // Map the PID outputs to servo positions (constrained between SERVO_MIN and SERVO_MAX)
    int yawPos   = constrain(map(outputYaw, -60, 60, SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX);
    int pitchPos = constrain(map(outputPitch, -60, 60, SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX);
    int rollPos  = constrain(map(outputRoll, -60, 60, SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX);

    // Update servo positions
    servoYaw.write(yawPos);
    servoPitch.write(pitchPos);
    servoRoll.write(rollPos);

    // Provide debug output every 200 ms
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 200) {
      lastUpdate = millis();
      Serial.print("Raw -> Yaw: "); Serial.print(rawYaw);
      Serial.print(" | Pitch: "); Serial.print(rawPitch);
      Serial.print(" | Roll: "); Serial.print(rawRoll);
      Serial.print(" || Filtered -> Yaw: "); Serial.print(filteredYaw);
      Serial.print(" | Pitch: "); Serial.print(filteredPitch);
      Serial.print(" | Roll: "); Serial.print(filteredRoll);
      Serial.print(" || Servos -> Yaw: "); Serial.print(yawPos);
      Serial.print(", Pitch: "); Serial.print(pitchPos);
      Serial.print(", Roll: "); Serial.println(rollPos);
    }

    // Log the acceleration data to the SD card every second
    if (millis() - lastLogTime >= 1000) {
      lastLogTime = millis();
      DateTime now = rtc.now();
      dataFile = SD.open("log.txt", FILE_WRITE);
      if (dataFile) {
        dataFile.print(now.year()); dataFile.print("/");
        dataFile.print(now.month()); dataFile.print("/");
        dataFile.print(now.day()); dataFile.print(" ");
        dataFile.print(now.hour()); dataFile.print(":");
        dataFile.print(now.minute()); dataFile.print(":");
        dataFile.print(now.second()); dataFile.print(", ");
        dataFile.print(aa.x); dataFile.print(", ");
        dataFile.print(aa.y); dataFile.print(", ");
        dataFile.println(aa.z);
        dataFile.close();
        Serial.println("Data written to SD card.");
      } else {
        Serial.println("Error opening file.");
      }
    }
  }
  delay(10);
}
