#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <MPU6050.h>

#define SD_CS 4  // Chip Select pin for SD card module

MPU6050 mpu;
File dataFile;

void setup() {
    Serial.begin(9600);
    Wire.begin();

    // Initialize SD card
    if (!SD.begin(SD_CS)) {
        Serial.println("SD Card initialization failed!");
        while (1);
    }
    Serial.println("SD Card initialized.");

    // Initialize MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1);
    }
}

void loop() {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    // Open file on SD card
    dataFile = SD.open("log.txt", FILE_WRITE);
    if (dataFile) {
        // Store XYZ acceleration data
        dataFile.print(ax); dataFile.print(", ");
        dataFile.print(ay); dataFile.print(", ");
        dataFile.println(az);

        dataFile.close();
        Serial.println("Data written to SD card.");
    } else {
        Serial.println("Error opening file.");
    }

    delay(1000);  // Log data every second
}