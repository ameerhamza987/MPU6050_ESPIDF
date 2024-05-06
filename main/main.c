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


#include <stdio.h>
#include "MPU6050.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Define I2C configuration
#define I2C_MASTER_SCL_IO 22                // I2C master clock GPIO pin
#define I2C_MASTER_SDA_IO 21                // I2C master data GPIO pin
#define I2C_MASTER_NUM I2C_NUM_0            // I2C port number
#define I2C_MASTER_FREQ_HZ 100000           // I2C master clock frequency

static const char *TAG = "MPU6050 Example";

// Create an instance of the MPU6050 class
MPU6050 mpu;

void initialize_i2c()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void initialize_mpu6050()
{
    if (!mpu.begin(0x68)) // Initialize MPU6050 with address 0x68
    {
        ESP_LOGE(TAG, "Failed to initialize MPU6050 sensor");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "MPU6050 sensor initialized successfully");
}

void app_main()
{
    initialize_i2c(); // Initialize I2C communication
    initialize_mpu6050(); // Initialize MPU6050 sensor

    float ax, ay, az; // Acceleration data
    float gx, gy, gz; // Gyroscope data
    float roll, pitch; // Roll and pitch angles
    float yaw; // Yaw angle
    float q0, q1, q2, q3; // Quaternion angles

    while (1)
    {
        // Read acceleration data
        mpu.readAccel(ax, ay, az);
        printf("Acceleration (m/s^2): X: %.2f, Y: %.2f, Z: %.2f\n", ax, ay, az);

        // Read gyroscope data
        mpu.readGyro(gx, gy, gz);
        printf("Gyroscope (deg/s): X: %.2f, Y: %.2f, Z: %.2f\n", gx, gy, gz);

        // Calculate roll and pitch
        roll = mpu.calcRoll(ax, ay, az);
        pitch = mpu.calcPitch(ax, ay, az);
        printf("Roll: %.2f, Pitch: %.2f\n", roll, pitch);

        // Estimate angles from quaternions
        mpu.AngelQuaternion(q0, q1, q2, q3, roll, pitch, yaw);
        printf("Roll (Quaternion): %.2f, Pitch (Quaternion): %.2f, Yaw: %.2f\n", roll, pitch, yaw);

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
    }
}
