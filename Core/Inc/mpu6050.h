
#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define MPU6050_ADDR_LOW  (0x68U << 1)  // 0xD0
#define MPU6050_ADDR_HIGH (0x69U << 1)  // 0xD2

#define MPU6050_REG_SMPLRT_DIV 0x19U
#define MPU6050_REG_CONFIG     0x1AU
#define MPU6050_REG_GYRO_CONFIG 0x1BU
#define MPU6050_REG_ACCEL_CONFIG 0x1CU
#define MPU6050_REG_ACCEL_XOUT_H 0x3BU
#define MPU6050_REG_PWR_MGMT_1 0x6BU
#define MPU6050_REG_WHO_AM_I   0x75U

#define MPU6050_GRAVITY        9.80665f
#define MPU6050_ACCEL_SENSITIVITY 16384.0f
#define MPU6050_GYRO_SENSITIVITY 131.0f

#define MPU6050_CALIBRATION_SAMPLES 1000U

typedef struct {
    // Orientation relative to the initial reference (degrees)
    float roll;
    float pitch;
    float yaw;

    // Acceleration in world coordinates (m/s^2)
    float world_ax;
    float world_ay;
    float world_az;

    // Estimated velocity (m/s)
    float vx;
    float vy;
    float vz;

    // Estimated displacement from starting position (m)
    float x;
    float y;
    float z;
} IMU_Estimate;

typedef struct {
    // Raw sensor values

    float ax_raw;
    float ay_raw;
    float az_raw;

    float gx_raw;
    float gy_raw;
    float gz_raw;


    // Calibrated physical measurements

    float ax;       // m/s^2
    float ay;       // m/s^2
    float az;       // m/s^2

    float gx;       // degrees/s
    float gy;       // degrees/s
    float gz;       // degrees/s

} MPU6050_Data;



typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint32_t address;

    // Offsets in raw sensor units

    float accel_offset_x;
    float accel_offset_y;
    float accel_offset_z;

    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;


    uint8_t initialized;
    uint8_t calibrated;
} MPU6050;

HAL_StatusTypeDef MPU6050_Init(
    MPU6050 *mpu,
    I2C_HandleTypeDef *hi2c,
    uint16_t address
);

HAL_StatusTypeDef MPU6050_Calibrate(
    MPU6050 *mpu,
    uint16_t samples
);

HAL_StatusTypeDef MPU6050_ReadData(
    MPU6050 *mpu,
    MPU6050_Data *data
);

void IMU_Update(
    IMU_Estimate *estimate,
    const MPU6050_Data *data,
    float dt
);

void MPU6050_ParseRawData(
    const uint8_t *buffer,
    MPU6050_Data *data
);


#endif
