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

#include "MPU6050.h"
#include <math.h>

// Define constants
#define PI 3.14159
#define RAD_TO_DEG  57.29577
#define AccelX_H_Addr  0x3B
#define AccelY_H_Addr  0x3D
#define AccelZ_H_Addr  0x3F
#define GyroX_H_Addr   0x43
#define GyroY_H_Addr   0x45
#define GyroZ_H_Addr   0x47

// Constructor
MPU6050::MPU6050() {}

// Initialize MPU6050
bool MPU6050::begin(uint8_t address)
{
    i2cAddr = address; // Store the I2C address

    // Initialize I2C communication
    esp_err_t ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        return false;
    }

    // Configure I2C parameters
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 21; // Replace with your SDA pin number
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = 22; // Replace with your SCL pin number
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000; // 100 kHz

    ret = i2c_param_config(I2C_NUM_0, &conf);
    if (ret != ESP_OK) {
        return false;
    }

    // Wake up MPU6050
    uint8_t data[2] = {0x6B, 0x00}; // Power Management 1 register, Wake up MPU6050
    ret = i2c_master_write(I2C_NUM_0, i2cAddr, data, 2, true);
    if (ret != ESP_OK) {
        return false;
    }

    return true;
}

// Read raw acceleration data from MPU6050 and convert it into usable form
void MPU6050::readAccel(float &accelX, float &accelY, float &accelZ)
{
    int16_t accelXRaw, accelYRaw, accelZRaw;
    readMPU6050(AccelX_H_Addr, accelXRaw); // Read X-axis accelerometer data
    readMPU6050(AccelY_H_Addr, accelYRaw); // Read Y-axis accelerometer data
    readMPU6050(AccelZ_H_Addr, accelZRaw); // Read Z-axis accelerometer data

    // Convert raw data to acceleration values
    accelX = accelXRaw / 16384.0; // Assuming MPU6050 set to +/-2g range
    accelY = accelYRaw / 16384.0;
    accelZ = accelZRaw / 16384.0;
}

// Read raw gyroscope data from MPU6050 and convert it into usable form
void MPU6050::readGyro(float &gyroX, float &gyroY, float &gyroZ)
{
    int16_t gyroXRaw, gyroYRaw, gyroZRaw;
    readMPU6050(GyroX_H_Addr, gyroXRaw); // Read X-axis gyroscope data
    readMPU6050(GyroY_H_Addr, gyroYRaw); // Read Y-axis gyroscope data
    readMPU6050(GyroZ_H_Addr, gyroZRaw); // Read Z-axis gyroscope data

    // Convert raw data to gyroscopic values
    gyroX = gyroXRaw / 131.0; // Assuming MPU6050 set to +/-250 degrees/s range
    gyroY = gyroYRaw / 131.0;
    gyroZ = gyroZRaw / 131.0;
}

// Calculate roll angle from accelerometer data
float MPU6050::calcRoll(float accelX, float accelY, float accelZ)
{
    return atan2(accelY, accelZ) * RAD_TO_DEG;
}

// Calculate pitch angle from accelerometer data
float MPU6050::calcPitch(float accelX, float accelY, float accelZ)
{
    return atan2(-accelX, sqrt((accelY * accelY) + (accelZ * accelZ))) * RAD_TO_DEG;
}

// Estimate angles from quaternions
void MPU6050::AngelQuaternion(float q0, float q1, float q2, float q3, float &roll, float &pitch, float &yaw)
{
    roll = atan2(2.0 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
    pitch = asin(2.0 * (q0 * q2 - q3 * q1));
    yaw = atan2(2.0 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));

    // Convert angles from radians to degrees
    roll *= RAD_TO_DEG;
    pitch *= RAD_TO_DEG;
    yaw *= RAD_TO_DEG;
}

// Private function to read data from MPU6050
void MPU6050::readMPU6050(uint8_t regAddr, int16_t &data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2cAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, regAddr, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(30 / portTICK_PERIOD_MS); // Delay to ensure data is ready

    uint8_t buffer[2];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2cAddr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &buffer[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &buffer[1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    // Combine the two received bytes into a 16-bit integer
    data = (buffer[0] << 8) | buffer[1];
}
