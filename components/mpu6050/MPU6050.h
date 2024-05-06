// MPU6050 Library for ESP-32
// Written by: Muhammad Ameer Hamza
// Date of creation: 07-05-2024
// Library Includes Following:
//  ---------------------------------------------------------
//  |   Functionality                         | Description |
//  ---------------------------------------------------------
//  |       readAccel                         |Read        |
//  |       readGyro                          |Read        |
//  |       calcRoll                          |Read        |
//  |       calcPitch                         |Read        |
//  |       AngelQuaternion                   |Read        |
//  ---------------------------------------------------------
//   mpu.readAccel(ax, ay, az);
//   mpu.readGyro(gx, gy, gz);
//   roll = mpu.calcRoll(ax, ay, az);
//   pitch = mpu.calcPitch(ax, ay, az);
//   mpu.AngelQuaternion(q0, q1, q2, q3, roll, pitch, yaw);
//
// Description:
// This library provides a set of functions to interface with the MPU6050 IMU sensor over I2C using ESP-32. It includes functions for reading acceleration data, reading gyroscope data, calculating roll and pitch, and estimating angles from quaternions. The library is designed to be simple to use and integrates seamlessly with the Arduino framework.
//
// Usage:
// - Include the library in your project:
//   #include "MPU6050.h"
//
// - Create an instance of the MPU6050 class with the default I2C address (0x68):
//   MPU6050 mpu;
//
// - Initialize the sensor in the setup() function:
//   mpu.begin();
//
// - Use the library functions to read data and calculate angles in the loop() function:
//   float ax, ay, az;
//   float gx, gy, gz;
//   float roll, pitch, yaw;
//
// Repository:
// The source code for this library can be found on GitHub/GitLab at:
// https://github.com/ameerhamza987/MPU6050_ESPIDF


#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"

#define MPU6050_ADDR_DEFAULT   0x68

class MPU6050 {
public:
    MPU6050();
    bool begin(uint8_t address = MPU6050_ADDR_DEFAULT);
    void readAccel(float &accelX, float &accelY, float &accelZ);
    void readGyro(float &gyroX, float &gyroY, float &gyroZ);
    float calcRoll(float accelX, float accelY, float accelZ);
    float calcPitch(float accelX, float accelY, float accelZ);
    void AngelQuaternion(float q0, float q1, float q2, float q3, float &roll, float &pitch, float &yaw);

private:
    uint8_t i2cAddr;
    void readMPU6050(uint8_t regAddr, int16_t &data);
};

#endif
