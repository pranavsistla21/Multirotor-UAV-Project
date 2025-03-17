#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <RTClib.h>
#include <MPU6050.h>

#define SD_CS 4  // Chip Select pin for SD card module

RTC_DS3231 rtc;
MPU6050 mpu;
File dataFile;

void setup() {
    Serial.begin(9600);
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
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1);
    }
}

void loop() {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    DateTime now = rtc.now();

    // Open file on SD card
    dataFile = SD.open("log.txt", FILE_WRITE);
    if (dataFile) {
        dataFile.print(now.year()); dataFile.print("/");
        dataFile.print(now.month()); dataFile.print("/");
        dataFile.print(now.day()); dataFile.print(" ");
        dataFile.print(now.hour()); dataFile.print(":");
        dataFile.print(now.minute()); dataFile.print(":");
        dataFile.print(now.second()); dataFile.print(", ");

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