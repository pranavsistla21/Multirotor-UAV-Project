#include<PID_v1.h>
#include <Wire.h>
#include <Servo.h>
#include <MPU6050_6Axis_MotionApps20.h>

MPU6050 mpu; // the setup for the MPU and DMP

Servo servoYaw,  servoPitch, servoRoll; // servo objects

// For every axis: Input - Filtered angle, Output - PID Correction, Setpoint - Desired Angle (which is = 0)
double inputYaw, outputYaw, setPointYaw = 0;
double inputPitch, outputPitch, setPointPitch = 0;
double inputRoll, outputRoll, setPointRoll = 0;

//LPF (Low Pass Filter) variables
double filteredYaw = 0, filteredPitch = 0, filteredRoll = 0;
const double LPF_ALPHA = 0.2; // rangle of LPF_ALPHA between 0&1. 0.2 is the smoothing factor for the LPF

// Parameters for PID tuning
double Kp = 2.0, Ki = 0.01, Kd = 0.05; // Adjustable
PID pidYaw(&inputYaw, &outputYaw, &setpointYaw, Kp, Ki, Kd, DIRECT);
PID pidYaw(&inputPitch, &outputPitch, &setpointPitch, Kp, Ki, Kd, DIRECT);
PID pidYaw(&inputRoll, &outputRoll, &setpointRoll, Kp, Ki, Kd, DIRECT);

// MPU6050 & DMP variable
uint8_t mpuIntStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3]; // ypr = yaw,pitch,roll

//Zerioing original position using offsets
double yawOffset = 0, pitchOffset = 0, rollOffset = 0;

// Servo limitation of movement
# define SERVO_MIN 0
#define SERVO_MAX 180

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Initialising MPU6050. Please Wait");
  mpu.initialize();
  if(!mpu.testConnection()) {
    Serial.println("MPU6050 connection failure");
    while(1);
  }

  Serial.println("Enabling DMP. Please wait");
  mpu.dmpinitialize();
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
  Serial.println("MPU6050 is Ready");

  // Stabalise MPU by giving it time to stabalise readings
  delay(1000);

  // First collect sample data to determine offset position
  const int NUM_SAMPLES = 50;
  double sumYaw = 0, sumPitch = 0, sumRoll = 0;
  int validSamples = 0;

  for (int i=0; i<NUM_SAMPLES; i++) {
    // checking for FIFO overflow or if data is available
    if ((mpuIntStatus & 0x10) || mpu.getFIFOCount() >= 1024) {
      mpu.resetFIFO();
      Serial.println("FIFO Overflow during calibration has occured! Resetting please wait");
      continue;
    }
    while (mpu.getFIFOCount() < packetSize) {
      // Wait until enough data is collected
      delay(5);
    }

    //Read Packet
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetyawPitchRoll(ypr, &q, &gravity);

    sumYaw += ypr[0]*180.0/M_PI;
    sumPitch += ypr[1]*180.0/M_PI;
    sumRoll += ypr[2]*180.0/M_PI;
    validSamples++;
    delay(10);
  }

  if (validSamples > 0) {
    yawOffset = sumYaw/validSamples;
    pitchOffset = sumpitch/validSamples;
    rollOffset = sumroll/validSamples;
  }

  serial.println("Offsets ---- Yaw: "); Serialprint(yawOffset);
  serial.println(", Pitch: "); Serialprint(pitchOffset);
  serial.println(", Roll: "); Serialprint(rollOffset);

  // Where servos are attached
  servoYaw.attach(11);
  servoPitch.attach(9);
  servoRoll.attach(10);

  //Initialising PID Controllers
  pidYaw.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);
  pidRoll.SetMode(AUTOMATIC);

  // Expanding output limits for a wider range of servo movements
  pidYaw.SetOutputLimits(-60, 60);
  pidPitch.SetOutputLimits(-60, 60);
  pidRoll.SetOutputLimits(-60, 60);

}

void loop() {
  //First check for FIFO overflow
  if((mpuIntStatus & 0x10) || mpu.getFIFOCount() >= 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO Overflow! Please wait for resetting");
    return;
  }
  //Read packet for FIFO overlfow
  if (mpu.getFIFOCount() >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmptGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Sensor values in degrees
    double rawYaw = (ypr[0]*180/M_PI) - yawOffset;
    double rawPitch = (ypr[0]*180/M_PI) - pitchOffset;
    double rawRoll = (ypr[0]*180/M_PI) - rollOffset;

    // Applying LPF
    filteredYaw = LPF_ALPHA*rawYaw +(1-LPF_ALPHA)*filteredYaw;
    filteredPitch = LPF_ALPHA*rawPitch +(1-LPF_ALPHA)*filteredPitch;
    filteredRoll = LPF_ALPHA*rawRoll +(1-LPF_ALPHA)*filteredRoll;

    // Setting filtered values as PID Inputs
    inputYaw = filteredYaw;
    inputPitch = filteredPitch;
    inputRoll = filteredRoll;

    //Computing PID corrections
    pidYaw.Compute();
    pidPitch.Compute();
    pidRoll.Compute();

    // Mapping PID outputs to servo positions
    int yawPos = constrain(map(outputYaw, -60,60, SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX);
    int pitchPos = constrain(map(outputYaw, -60,60, SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX);
    int rollPos = constrain(map(outputYaw, -60,60, SERVO_MIN, SERVO_MAX), SERVO_MIN, SERVO_MAX);

    // Writing positions of servos
    servoYaw.write(yawPos);
    servoPitch.write(pitchPos);
    servoRoll.write(rollPos);

    // debug output in this instance every 200ms
    static unsigned long lastUpdate = 0;
    if(millis() - lastUpdate > 200) {
      lastUpdate = millis();
      Serial.print("Raw --- Yaw: "); Serial.print(rawYaw);
      Serial.print(" | Pitch: "); Serial.print(rawPitch);
      Serial.print(" | Roll: "); Serial.print(rawRoll);
      Serial.print(" ||| Filtered --- Yaw: "); Serial.print(filteredYaw);
      Serial.print(" | Pitch: "); Serial.print(filteredPitch);
      Serial.print(" | Roll: "); Serial.print(filteredRoll);
      Serial.print(" ||| Servos --- Yaw: "); Serial.print(yawPos);
      Serial.print(" | Pitch: "); Serial.print(pitchPos);
      Serial.print(" | Roll: "); Serial.print(rollPos);

    }

  }
  delay(10);
}












































