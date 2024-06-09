/*
 * MPUXX50.h
 *
 *  Created on: Feb 27, 2022
 *      Author: MarkSherstan
 */

#ifndef MPUXX50_H_
#define MPUXX50_H_

// Libs
#include "main.h"
#include <stdint.h>
#include <math.h>

// Constants
#define RAD2DEG                   57.2957795131

// Defines
#define WHO_AM_I_6050_ANS         0x68
#define WHO_AM_I_9250_ANS         0x71
#define WHO_AM_I                  0x75
#define INT_PIN_CFG               0x37
#define AD0_LOW                   0x68
#define AD0_HIGH                  0x69
#define CONFIG                    0x1A
#define ACCEL_CONFIG1             0x1C
#define ACCEL_CONFIG2             0x1D
#define GYRO_CONFIG               0x1B
#define PWR_MGMT_1                0x6B
#define ACCEL_XOUT_H              0x3B
#define I2C_TIMOUT_MS             1000

// Magnetometer defines
#define MAG_WHO_AM_I_REG          0x00  // magnetometer `who am i` register address
#define WHO_AM_I_MAG_RSP          0x48  // magnetometer `who am i` response

#define MAG_MODE_POWERDOWN        0x0
#define MAG_MODE_SINGLE           0x1
#define MAG_MODE_CONTINUOUS_8HZ   0x2
#define MAG_MODE_EXTERNAL         0x4
#define MAG_MODE_CONTINUOUS_100HZ 0x6
#define MAG_MODE_SELFTEST         0x8
#define MAG_MODE_FUSEROM          0xF
#define MAG_ADDRESS               0x0C
#define MAG_HXL                   0x03   // X axis value register
#define MAG_CNTL                  0x0A   // magnetometer control register address
#define MAG_ASAX                  0x10   // magnetometer x sensivity value 
#define MAG_ST1                   0x02   // magnetometer status ST1 reg 

#define ACCEL_DLPF_184            0x01
#define ACCEL_DLPF_92             0x02
#define ACCEL_DLPF_41             0x03
#define ACCEL_DLPF_20             0x04
#define ACCEL_DLPF_10             0x05
#define ACCEL_DLPF_5              0x06
#define ACCEL_DLPF_OFF            0x00
// 460 Hz 0x07 is not allowed 
#define GYRO_DLPF_250             0x00
#define GYRO_DLPF_184             0x01
#define GYRO_DLPF_92              0x02
#define GYRO_DLPF_41              0x03
#define GYRO_DLPF_20              0x04
#define GYRO_DLPF_10              0x05
#define GYRO_DLPF_5               0x06
#define GYRO_DLPF_OFF             0x07
// 3600 Hz Same

#define MAG_LENGTH_MEASURE_OFFSET 4


enum magnetometerMeasurementLength
{
    MAG_14BIT_MEASURE = 0,
    MAG_16BIT_MEASURE
};


// Full scale ranges
enum gyroscopeFullScaleRange
{
    GFSR_250DPS,
    GFSR_500DPS,
    GFSR_1000DPS,
    GFSR_2000DPS
};

enum accelerometerFullScaleRange
{
    AFSR_2G,
    AFSR_4G,
    AFSR_8G,
    AFSR_16G
};

#define MFSR 4912
// Structures
typedef struct
{
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz, t;
} RawData;

typedef struct SensorData
{
    float ax, ay, az, gx, gy, gz, mx, my, mz, t;
} SensorData;

typedef struct
{
    float x, y, z; // x, y, and z offsets
} GyroCal;

typedef struct
{
    float r, p, y; // roll, pitch, yaw
} Attitude;


// Variables
extern RawData rawData;
extern SensorData sensorData;
extern GyroCal gyroCal;
extern Attitude attitude;

extern float dt, tau;
extern float aScaleFactor, gScaleFactor, magScaleXFactor, magScaleYFactor, magScaleZFactor;


// Functions
uint8_t MPU_begin(I2C_HandleTypeDef *I2Cx, uint8_t addr,
                uint8_t aScale, uint8_t gScale, 
                float tau, float dt);
void MPU_calibrateGyro(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints);
void MPU_calcAttitude(I2C_HandleTypeDef *I2Cx);
void MPU_readRawData(I2C_HandleTypeDef *I2Cx);
void MPU_readProcessedData(I2C_HandleTypeDef *I2Cx);
void MPU_writeGyroFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t gScale);
void MPU_writeAccFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t aScale);
void MPU_writeMagFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t mScale);
int MPU_setFilt(I2C_HandleTypeDef *, uint8_t, uint8_t);
void MPU_initMag(I2C_HandleTypeDef *, uint8_t);
#endif /* MPUXX50_H_ */
