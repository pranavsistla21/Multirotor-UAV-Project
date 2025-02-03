# include <Wire.h> //I2C communcation protocol
# include <MPU6050.h>
# include <Servo.h>

MPU6050 mpu; // intialises the MPU
Servo yaw_Servo, pitch_Servo, roll_Servo; //initialises the servos (x,y,z)

float Kp = 2.4, Ki = 0.02, Kd = 1.1; //PID Gain values

// PID variables for the all axeis
float Previous_Error_X = 0, Previous_Error_Y = 0, Previous_Error_Z = 0;
float Integral_X = 0, Integral_Y = 0, Integral_Z = 0;

// The MPU will aim to keep the servos at 90 degress as it's the inital condition
int Set_Point_X = 90;
int Set_Point_Y = 90;
int Set_Point_Z = 90;

float Offset_X = 0, Offset_Y = 0, Offset_Z = 0; // stores intial offset values

void setup () {
    Serial.begin(115200); // Begins serial communication at 115200 baud rate
    Wire.begin(); //Begins I2C communcation protocol to communcate with the MPU

    mpu.initialize();
    if (!mpu.testConnection()) { // checks connection of MPU to arduino
        Serial.println("Connection with MPU6050 not established");
        while (1); // Stops program if connection not established
    }

    // Attach servos to the following ports
    yaw_Servo.attach(9);
    pitch_Servo.attach(10);
    roll_Servo.attach(11);

    // Centering the servos
    yaw_Servo.write(Set_Point_X);
    pitch_Servo.write(Set_Point_Y);
    roll_Servo.write(Set_Point_Z);

    // GIVES 5 seconds to place place gimbal in desired orientation
    Serial.println("Place gimbal in desired orentation");
    delay(5000);

    // The initial position is considered the reference point
    int16_t ax, ay, az; // int16_t is a 16-bit data type that stores +&- numbers; the MPU gives 16-bit +/- integers. ax, ay, az store the accelerometer values
    float sumX = 0, sumY = 0, sumZ = 0; //floating data point storing decimals

    for (int i = 0; i<200; i++) { // initialising loop variable i=0 until i<200, i++ means each interation ,i, increases by 1
        mpu.getAcceleration(&ax, &ay, &az);
        sumX += atan2(ay, az) * 180/PI; // converting the acceleration data into angles
        sumY += atan2(ax, az) * 180/PI; // converting the acceleration data into angles
        sumZ += atan2(ax, ay) * 180/PI; // converting the acceleration data into angles
        // += Adds up the angle values
        delay(5); // delay between readings for stabalisation
    }

    Offset_X = sumX / 200;
    Offset_Y = sumY / 200;
    Offset_Z = sumZ / 200;

    Serial.println("Initial position established");
    Serial.print("Offest X: "); Serial.println(Offset_X);
    Serial.print("Offest Y: "); Serial.println(Offset_Y);
    Serial.print("Offest Z: "); Serial.println(Offset_Z);

    Serial.println("Stabalisation initiated after calibration");

}

void loop () { // performs actions and doesn't return any values
    int16_t ax, ay, az, gx, gy, gz; // a for accelerometer, g for gyroscope
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //'&' means function will directly modify the variable

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
    Serial.print(" | Roll: "); Serial.print(output_Z);

    delay(15); //delay between each consecutive set of readings

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


