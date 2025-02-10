#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <Servo.h>

MPU6050 mpu;
Servo servo_yaw;
Servo servo_pitch;
Servo servo_roll;

// Offsets for calibration
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

// Function for smoothing
float smooth(float current, float previous, float alpha) {
  return previous + alpha * (current - previous);
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);

  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println(F("MPU Connection is Successful."));
  } else {
    Serial.println(F("MPU Connection failure. Check wires."));
    while (1);
  }

  // Attach the servos to the corresponding PWM pins
  servo_yaw.attach(4);
  servo_pitch.attach(5);
  servo_roll.attach(6);

  // Starting positions at centre position
  servo_yaw.write(90);
  servo_pitch.write(90);
  servo_roll.write(90);
  delay(1000); // 1 sec to reach the positions
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Apply calibration offsets
  ax -= ax_offset;
  ay -= ay_offset;
  az -= az_offset;
  gx -= gx_offset;
  gy -= gy_offset;
  gz -= gz_offset;

  // Prevent division by zero in roll calculation
  float denominator = sqrt(ay * ay + az * az);
  float roll = (denominator > 0.01) ? atan2(ax, denominator) * 180 / M_PI * 2 : 0; // Roll axis inverted and increased sensitivity
  Serial.print("Raw roll: "); Serial.println(roll); // Debugging roll values

  float pitch = (atan2(ay, az) * 180 / M_PI) * 2; // Pitch sensitivity increased
  float yaw = (gx / 131.0) * 3; // Yaw sensitivity increased

  // Smooth sensor data
  static float smoothed_yaw = 0, smoothed_pitch = 0, smoothed_roll = 0;
  smoothed_yaw = smooth(yaw, smoothed_yaw, 0.01);
  smoothed_pitch = smooth(pitch, smoothed_pitch, 0.01);
  smoothed_roll = smooth(roll, smoothed_roll, 0.01);

  // Map sensor values to servo positions with amplified sensitivity
  int yaw_value = map(smoothed_yaw, -90, 90, 0, 180);
  int pitch_value = map(smoothed_pitch, -45, 45, 180, 0); // Pitch movements narrowed for sensitivity
  int roll_value = map(smoothed_roll, -30, 30, 0, 180);   // Roll movements narrowed

  // Write values to servos
  servo_yaw.write(constrain(yaw_value, 0, 180));
  servo_pitch.write(constrain(pitch_value, 0, 180));
  servo_roll.write(constrain(roll_value, 0, 180));

  // Debugging smoothed values
  Serial.print("Yaw: "); Serial.print(smoothed_yaw);
  Serial.print(" | Pitch: "); Serial.print(smoothed_pitch);
  Serial.print(" | Roll: "); Serial.println(smoothed_roll);

  delay(100);
}
