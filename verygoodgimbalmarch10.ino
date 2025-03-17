#include <Wire.h>
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>

// ----- MPU6050 and DMP Setup -----
MPU6050 mpu;

// ----- Servo Objects -----
Servo servoYaw, servoPitch, servoRoll;

// ----- PID Variables -----
// For each axis: input (filtered angle), output (PID correction), setpoint (desired angle = 0)
double inputYaw, outputYaw, setpointYaw = 0;
double inputPitch, outputPitch, setpointPitch = 0;
double inputRoll, outputRoll, setpointRoll = 0;

// ----- Low-Pass Filter Variables -----
double filteredYaw = 0, filteredPitch = 0, filteredRoll = 0;
const double LPF_ALPHA = 0.2;  // 0 < LPF_ALPHA < 1

// ----- PID Tuning Parameters -----
double Kp = 2.0, Ki = 0.01, Kd = 0.05;  // Adjust as needed
PID pidYaw(&inputYaw, &outputYaw, &setpointYaw, Kp, Ki, Kd, DIRECT);
PID pidPitch(&inputPitch, &outputPitch, &setpointPitch, Kp, Ki, Kd, DIRECT);
PID pidRoll(&inputRoll, &outputRoll, &setpointRoll, Kp, Ki, Kd, DIRECT);

// ----- MPU6050 DMP Variables -----
uint8_t mpuIntStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];  // [yaw, pitch, roll]

// ----- Offsets for Zeroing Initial Position -----
double yawOffset = 0, pitchOffset = 0, rollOffset = 0;

// ----- Servo Limits -----
#define SERVO_MIN 0
#define SERVO_MAX 180

void setup() {
  Serial.begin(115200);
  Wire.begin();

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

  // ----- Warm-up / Stabilize -----
  // Give the MPU6050 some time to produce stable readings
  delay(1000);

  // ----- Collect Multiple Samples to Determine Offsets -----
  const int NUM_SAMPLES = 50;
  double sumYaw = 0, sumPitch = 0, sumRoll = 0;
  int validSamples = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    // Check for FIFO overflow or data availability
    if ((mpuIntStatus & 0x10) || mpu.getFIFOCount() >= 1024) {
      mpu.resetFIFO();
      Serial.println("FIFO Overflow during calibration! Resetting...");
      continue;
    }
    while (mpu.getFIFOCount() < packetSize) {
      // Wait until we have enough data
      delay(5);
    }

    // Read a packet
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
  
  // ----- Attach Servos -----
  servoYaw.attach(11);
  servoPitch.attach(9);
  servoRoll.attach(10);
  
  // ----- Initialize PID Controllers -----
  pidYaw.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);
  pidRoll.SetMode(AUTOMATIC);
  
  // Expand the output limits to -60...+60 for a wider servo range
  pidYaw.SetOutputLimits(-60, 60);
  pidPitch.SetOutputLimits(-60, 60);
  pidRoll.SetOutputLimits(-60, 60);
}

void loop() {
  // ----- Check for FIFO Overflow -----
  if ((mpuIntStatus & 0x10) || mpu.getFIFOCount() >= 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO Overflow! Resetting...");
    return;
  }
  
  // ----- Read a Packet from the FIFO -----
  if (mpu.getFIFOCount() >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    // ----- Raw sensor values (in degrees) -----
    double rawYaw   = (ypr[0] * 180.0 / M_PI) - yawOffset;
    double rawPitch = (ypr[1] * 180.0 / M_PI) - pitchOffset;
    double rawRoll  = (ypr[2] * 180.0 / M_PI) - rollOffset;
    
    // ----- Apply Low-Pass Filtering -----
    filteredYaw   = LPF_ALPHA * rawYaw   + (1 - LPF_ALPHA) * filteredYaw;
    filteredPitch = LPF_ALPHA * rawPitch + (1 - LPF_ALPHA) * filteredPitch;
    filteredRoll  = LPF_ALPHA * rawRoll  + (1 - LPF_ALPHA) * filteredRoll;
    
    // ----- Set Filtered Values as PID Inputs -----
    inputYaw   = filteredYaw;
    inputPitch = filteredPitch;
    inputRoll  = filteredRoll;
    
    // ----- Compute PID Corrections -----
    pidYaw.Compute();
    pidPitch.Compute();
    pidRoll.Compute();
    
    // ----- Map PID Outputs to Servo Positions -----  
    int yawPos   = constrain(map(outputYaw, -60, 60, SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX);
    int pitchPos = constrain(map(outputPitch, -60, 60, SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX);
    int rollPos  = constrain(map(outputRoll, -60, 60, SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX);
    
    // ----- Write Servo Positions -----
    servoYaw.write(yawPos);
    servoPitch.write(pitchPos);
    servoRoll.write(rollPos);
    
    // ----- Debug Output (every 200 ms) -----
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
  }
  delay(10);
}

