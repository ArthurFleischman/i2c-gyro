//
// Created by Arthur Fleischman on 10/12/26.
//


#include "mpu6050.h"

#include <math.h>
#include <string.h>

#define MPU6050_TIMEOUT_MS 100U
#define RAD_TO_DEG 57.2957795f
#define GRAVITY     9.80665f
#define FILTER_ALPHA 0.98f

static int16_t read_int16_be(const uint8_t *buffer)
{
    uint16_t raw = ((uint16_t)buffer[0] << 8)
                 | (uint16_t)buffer[1];

    return (int16_t)raw;
}

void MPU6050_ParseRawData(
    const uint8_t *buffer,
    MPU6050_Data *data
) {
    if (buffer == 0 || data == 0) {
        return;
    }

    data->ax_raw = read_int16_be(&buffer[0]);
    data->ay_raw = read_int16_be(&buffer[2]);
    data->az_raw = read_int16_be(&buffer[4]);

    // Bytes 6-7 contain temperature.
    data->gx_raw = read_int16_be(&buffer[8]);
    data->gy_raw = read_int16_be(&buffer[10]);
    data->gz_raw = read_int16_be(&buffer[12]);
}

static HAL_StatusTypeDef write_reg(
    MPU6050 *mpu,
    uint8_t reg,
    uint8_t value
) {
    return HAL_I2C_Mem_Write(
        mpu->hi2c,
        mpu->address,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        MPU6050_TIMEOUT_MS
    );
}

static HAL_StatusTypeDef read_regs(
    MPU6050 *mpu,
    uint8_t reg,
    uint8_t *buffer,
    uint16_t length
) {
    return HAL_I2C_Mem_Read(
        mpu->hi2c,
        mpu->address,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buffer,
        length,
        MPU6050_TIMEOUT_MS
    );
}

HAL_StatusTypeDef MPU6050_Init(
    MPU6050 *mpu,
    I2C_HandleTypeDef *hi2c,
    uint16_t address
) {
    uint8_t who_am_i;

    if (mpu == NULL || hi2c == NULL) {
        return HAL_ERROR;
    }

    memset(mpu, 0, sizeof(*mpu));

    mpu->hi2c = hi2c;
    mpu->address = address;

    // Verify the sensor identity.
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
            hi2c,
            MPU6050_ADDR_LOW,
            0x75,
            I2C_MEMADD_SIZE_8BIT,
            &who_am_i,
            1,
            100);

    if (status != HAL_OK) {
        return status;
    }

    if ((who_am_i & 0x7EU) != 0x68U) {
        return HAL_ERROR;
    }

    // Reset the sensor.
    status = write_reg(mpu, MPU6050_REG_PWR_MGMT_1, 0x80U);
    if (status != HAL_OK) return status;

    HAL_Delay(100);

    // Wake up; select the X gyro PLL as the clock source.
    status = write_reg(mpu, MPU6050_REG_PWR_MGMT_1, 0x01U);
    if (status != HAL_OK) return status;

    HAL_Delay(10);

    // Enable the digital low-pass filter.
    // DLPF_CFG = 3: approximately 44 Hz gyro bandwidth.
    status = write_reg(mpu, MPU6050_REG_CONFIG, 0x03U);
    if (status != HAL_OK) return status;

    // Sample rate = 1 kHz / (1 + 4) = 200 Hz.
    status = write_reg(mpu, MPU6050_REG_SMPLRT_DIV, 0x04U);
    if (status != HAL_OK) return status;

    // Gyroscope: +/-250 degrees/s.
    status = write_reg(mpu, MPU6050_REG_GYRO_CONFIG, 0x00U);
    if (status != HAL_OK) return status;

    // Accelerometer: +/-2 g.
    status = write_reg(mpu, MPU6050_REG_ACCEL_CONFIG, 0x00U);
    if (status != HAL_OK) return status;

    // Enable all sensor axes.
    status = write_reg(mpu, 0x6CU, 0x00U);
    if (status != HAL_OK) return status;

    HAL_Delay(50);

    mpu->initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Calibrate(
    MPU6050 *mpu,
    uint16_t samples
) {
    if (mpu == NULL || !mpu->initialized || samples == 0U) {
        return HAL_ERROR;
    }

    int64_t ax_sum = 0;
    int64_t ay_sum = 0;
    int64_t az_sum = 0;

    int64_t gx_sum = 0;
    int64_t gy_sum = 0;
    int64_t gz_sum = 0;

    uint8_t buffer[14];

    // IMPORTANT:
    // Keep the sensor stationary and level, with +Z pointing upward.
    // The expected accelerometer readings are approximately
    // (0, 0, +1 g) in this orientation.
    MPU6050_Data raw_data = {0};
    for (uint16_t i = 0; i < samples; i++) {
        HAL_StatusTypeDef status = read_regs(
            mpu,
            MPU6050_REG_ACCEL_XOUT_H,
            buffer,
            sizeof(buffer)
        );

        if (status != HAL_OK) {
            mpu->calibrated = 0U;
            return status;
        }

        MPU6050_ParseRawData(buffer,&raw_data);

        ax_sum += (uint16_t)raw_data.ax_raw;
        ay_sum += (uint16_t)raw_data.ay_raw;
        az_sum += (uint16_t)raw_data.az_raw;

        gx_sum += (uint16_t)raw_data.gx_raw;
        gy_sum += (uint16_t)raw_data.gy_raw;
        gz_sum += (uint16_t)raw_data.gz_raw;

        HAL_Delay(2);
    }

    mpu->accel_offset_x = (float)ax_sum / (float)samples;
    mpu->accel_offset_y = (float)ay_sum / (float)samples;

    // Preserve the +1 g gravity component.
    mpu->accel_offset_z =
        (float)az_sum / (float)samples - MPU6050_ACCEL_SENSITIVITY;

    // A stationary gyro should read approximately zero.
    mpu->gyro_offset_x = (float)gx_sum / (float)samples;
    mpu->gyro_offset_y = (float)gy_sum / (float)samples;
    mpu->gyro_offset_z = (float)gz_sum / (float)samples;

    mpu->calibrated = 1U;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadData(
    MPU6050 *mpu,
    MPU6050_Data *data
) {
    if (mpu == NULL || data == NULL || !mpu->initialized) {
        return HAL_ERROR;
    }

    uint8_t buffer[14];

    HAL_StatusTypeDef status = read_regs(
        mpu,
        MPU6050_REG_ACCEL_XOUT_H,
        buffer,
        sizeof(buffer)
    );

    if (status != HAL_OK) {
        return status;
    }

    // Decode big-endian signed 16-bit values.
    MPU6050_ParseRawData(buffer,data);

    // Convert acceleration from raw counts to m/s^2.
    data->ax = (data->ax_raw - (mpu->calibrated ? mpu->accel_offset_x : 0.0f))
         / MPU6050_ACCEL_SENSITIVITY * MPU6050_GRAVITY;

    data->ay = (data->ay_raw - (mpu->calibrated ? mpu->accel_offset_y : 0.0f))
             / MPU6050_ACCEL_SENSITIVITY * MPU6050_GRAVITY;

    data->az = (data->az_raw - (mpu->calibrated ? mpu->accel_offset_z : 0.0f))
             / MPU6050_ACCEL_SENSITIVITY * MPU6050_GRAVITY;

    // Convert angular velocity to degrees/s.
    data->gx = (data->gx_raw - (mpu->calibrated ? mpu->gyro_offset_x : 0.0f))
             / MPU6050_GYRO_SENSITIVITY;

    data->gy = (data->gy_raw - (mpu->calibrated ? mpu->gyro_offset_y : 0.0f))
             / MPU6050_GYRO_SENSITIVITY;

    data->gz = (data->gz_raw - (mpu->calibrated ? mpu->gyro_offset_z : 0.0f))
             / MPU6050_GYRO_SENSITIVITY;

    return HAL_OK;
}

void IMU_Update(
    IMU_Estimate *e,
    const MPU6050_Data *d,
    float dt
) {
    if (e == NULL || d == NULL || dt <= 0.0f) {
        return;
    }

    // 1. Accelerometer-based tilt (degrees).
    // Assumes sensor Z points upward when level.
    const float roll_acc =
        atan2f(d->ay, d->az) * RAD_TO_DEG;

    const float pitch_acc =
        atan2f(
            -d->ax,
            sqrtf(d->ay * d->ay + d->az * d->az)
        ) * RAD_TO_DEG;

    // 2. Complementary filter.
    // Gyroscope values must be in degrees/second.
    // dt must be in seconds.
    e->roll =
        FILTER_ALPHA * (e->roll + d->gx * dt)
        + (1.0f - FILTER_ALPHA) * roll_acc;

    e->pitch =
        FILTER_ALPHA * (e->pitch + d->gy * dt)
        + (1.0f - FILTER_ALPHA) * pitch_acc;

    // Yaw is gyro-integrated and will drift without
    // an external heading reference.
    e->yaw += d->gz * dt;

    // 3. Convert angles to radians.
    const float roll  = e->roll  / RAD_TO_DEG;
    const float pitch = e->pitch / RAD_TO_DEG;
    const float yaw   = e->yaw   / RAD_TO_DEG;

    const float cr = cosf(roll);
    const float sr = sinf(roll);
    const float cp = cosf(pitch);
    const float sp = sinf(pitch);
    const float cy = cosf(yaw);
    const float sy = sinf(yaw);

    // 4. Rotate acceleration into the world frame.
    // Assumes sensor axes match the rotation convention.
    const float ax = d->ax;
    const float ay = d->ay;
    const float az = d->az;

    const float world_ax =
        cp * cy * ax
        + (sr * sp * cy - cr * sy) * ay
        + (cr * sp * cy + sr * sy) * az;

    const float world_ay =
        cp * sy * ax
        + (sr * sp * sy + cr * cy) * ay
        + (cr * sp * sy - sr * cy) * az;

    const float world_az =
        -sp * ax
        + sr * cp * ay
        + cr * cp * az;

    e->world_ax = world_ax;
    e->world_ay = world_ay;
    e->world_az = world_az - GRAVITY;

    // 5. Integrate acceleration into velocity.
    const float old_vx = e->vx;
    const float old_vy = e->vy;
    const float old_vz = e->vz;

    e->vx += e->world_ax * dt;
    e->vy += e->world_ay * dt;
    e->vz += e->world_az * dt;

    // 6. Integrate velocity into position using
    // the average of the old and new velocities.
    e->x += 0.5f * (old_vx + e->vx) * dt;
    e->y += 0.5f * (old_vy + e->vy) * dt;
    e->z += 0.5f * (old_vz + e->vz) * dt;
}
