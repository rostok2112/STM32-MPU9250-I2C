/*
 * MPUXX50.c
 *
 *  Created on: Feb 27, 2022
 *      Author: MarkSherstan
 */

#include "MPUXX50.h"

// Variables
RawData rawData;
SensorData sensorData;
GyroCal gyroCal;
Attitude attitude;

float dt, tau;
float aScaleFactor, gScaleFactor, mScaleXFactor, mScaleYFactor, mScaleZFactor;

static uint8_t _addr;
static uint8_t isMPU9250;

// Functions

/// @brief Set the digilat low pass filter.
/// @param I2Cx Pointer to I2C structure config.
/// @param accel_bandwidth target accelerometer DLPF bandwidth.
/// @param gyro_bandwidth  target gyroscope DLPF bandwidth.
int MPU_setFilt(I2C_HandleTypeDef *I2Cx, uint8_t accel_bandwidth, uint8_t gyro_bandwidth) {

    // Configure accelerometer bandwidth
    if (HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG2, 1, &accel_bandwidth, 1, I2C_TIMOUT_MS) != HAL_OK) {
        return -1;
    }
    // Configure gyroscope bandwidth
    if (HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &gyro_bandwidth, 1, I2C_TIMOUT_MS) != HAL_OK) {
        return -1;
    }

    return 0;
}


/// @brief Initialize the magnetometer.
/// @param I2Cx Pointer to I2C structure config.
/// @param mMode a mode of magnitometer work.
void MPU_initMag(I2C_HandleTypeDef *I2Cx, uint8_t mMode) {
    float mScaleFactor = MFSR / 32768.0 ;  // magnetometer FSR is constant

    uint8_t writeData;

  //First extract the factory calibration for each magnetometer axis
  // x/y/z gyro calibration data stored here
  uint8_t rawMagCalData[3];

  //Power down magnetometer
  writeData = 0x00;
  HAL_I2C_Mem_Write(I2Cx, MAG_ADDRESS, MAG_CNTL, 1, &writeData, 1, I2C_TIMOUT_MS);
  HAL_Delay(100);

  writeData = MAG_MODE_FUSEROM;

  HAL_I2C_Mem_Write(I2Cx, MAG_ADDRESS, MAG_CNTL, 1, &writeData, 1, I2C_TIMOUT_MS);// Enter Fuse ROM access mode
  HAL_Delay(100);

  HAL_I2C_Mem_Read(I2Cx, MAG_ADDRESS, MAG_ASAX, 1, &rawMagCalData[0], 3, I2C_TIMOUT_MS);// Read the x-, y-, and z-axis calibration values

  mScaleXFactor = ((((float)rawMagCalData[0]) - 128.0f)/(256.0f) + 1.0f) * mScaleFactor; // Return x-axis sensitivity adjustment values, etc. in  uT
  mScaleYFactor = ((((float)rawMagCalData[1]) - 128.0f)/(256.0f) + 1.0f) * mScaleFactor; // micro Tesla
  mScaleZFactor = ((((float)rawMagCalData[2]) - 128.0f)/(256.0f) + 1.0f) * mScaleFactor; // micro Tesla

  writeData = 0x00;
  HAL_I2C_Mem_Write(I2Cx, MAG_ADDRESS, MAG_CNTL, 1, &writeData, 1, I2C_TIMOUT_MS);// Power down magnetometer
  HAL_Delay(100);

  // Configure the magnetometer for several meausrement mode
  writeData = (MAG_16BIT_MEASURE << MAG_LENGTH_MEASURE_OFFSET) | mMode;
  HAL_I2C_Mem_Write(I2Cx, MAG_ADDRESS, MAG_CNTL, 1, &writeData, 1, I2C_TIMOUT_MS);// Set magnetometer data resolution and sample ODR
  HAL_Delay(10);


}

/// @brief Set the IMU address, check for connection, reset IMU, and set full range scale.
/// @param I2Cx Pointer to I2C structure config.
/// @param addr Hex address based on AD0 pin - 0x68 low or 0x69 high.
/// @param aScale Set accelerometer full scale range: 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.
/// @param gScale Set gyroscope full scale range: 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s.
/// @param tau Set tau value for the complementary filter (typically 0.98).
/// @param dt Set sampling rate in seconds determined by the timer interrupt.
uint8_t MPU_begin(I2C_HandleTypeDef *I2Cx, uint8_t addr, 
                uint8_t aScale, uint8_t gScale, 
                float tau, float dt)
{
    // Save values
    _addr = addr << 1;
    tau = tau;
    dt = dt;

    // Initialize variables
    uint8_t readData;
    uint8_t writeData;

    // Confirm device
    HAL_I2C_Mem_Read(I2Cx, _addr, WHO_AM_I, 1, &readData, 1, I2C_TIMOUT_MS);


    // TODO: If 9250 or 6050 fails could it trigger the opposite check???
    isMPU9250 = readData == WHO_AM_I_9250_ANS;
    if (isMPU9250 || (readData == WHO_AM_I_6050_ANS))
    {
        // Startup / reset the sensor
        writeData = 0x00;
        HAL_I2C_Mem_Write(I2Cx, _addr, PWR_MGMT_1, 1, &writeData, 1, I2C_TIMOUT_MS);

        // Set the full scale ranges
        MPU_writeAccFullScaleRange(I2Cx, aScale);
        MPU_writeGyroFullScaleRange(I2Cx, gScale);

        MPU_setFilt(I2Cx, ACCEL_DLPF_184, GYRO_DLPF_184);
        if(isMPU9250) {
            writeData = 0x22;
		    HAL_I2C_Mem_Write(I2Cx, _addr, INT_PIN_CFG, 1, &writeData, 1, I2C_TIMOUT_MS);

            //Read the WHO_AM_I register of the magnetometer
            HAL_I2C_Mem_Read(I2Cx, MAG_ADDRESS, MAG_WHO_AM_I_REG, 1, &readData, 1, I2C_TIMOUT_MS);  // Read WHO_AM_I register for AK8963
            //if(readData == WHO_AM_I_MAG_RSP) {
                //Get magnetometer calibration from AK8963 ROM

            uint8_t i2cData = 0x02;
            HAL_I2C_Mem_Write(I2Cx, AD0_LOW << 1, INT_PIN_CFG, 1, &i2cData, 1, I2C_TIMOUT_MS);

            // Configure the magnetometer for continuous read
            i2cData = MAG_MODE_CONTINUOUS_8HZ;

            HAL_I2C_Mem_Write(I2Cx, MAG_ADDRESS << 1, MAG_CNTL, 1, &i2cData, 1, I2C_TIMOUT_MS);
            //    MPU_initMag(I2Cx, MAG_MODE_CONTINUOUS_100HZ);  // Initialize device for active mode read of magnetometer

                // TODO: add calibration for magnitometer
            //}
        }
        

        return 1;
    }
    else
    {
        return 0;
    }
}

/// @brief Set the accelerometer full scale range.
/// @param I2Cx Pointer to I2C structure config.
/// @param aScale Set 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.
void MPU_writeAccFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t aScale)
{
    // Variable init
    uint8_t select;

    // Set the value
    switch (aScale)
    {
    case AFSR_2G:
        aScaleFactor = 16384.0;  // 32768 (max signed 4 bytes value) // 2
        select = 0x00;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG1, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_4G:
        aScaleFactor = 8192.0;  // 32768 (max signed 4 bytes value) // 4
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG1, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_8G:  // 32768 (max signed 4 bytes value) // 8
        aScaleFactor = 4096.0;
        select = 0x10;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG1, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_16G:  // 32768 (max signed 4 bytes value) // 16
        aScaleFactor = 2048.0;
        select = 0x18;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG1, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        aScaleFactor = 8192.0;  // 32768 (max signed 4 bytes value) // 4
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG1, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }
}

/// @brief Set the gyroscope full scale range.
/// @param I2Cx Pointer to I2C structure config.
/// @param gScale Set 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s.
void MPU_writeGyroFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t gScale)
{
    // Variable init
    uint8_t select;

    // Set the value
    switch (gScale)
    {
    case GFSR_250DPS:
        gScaleFactor = 131.0;  // 32768 (max signed 4 bytes value) // 250
        select = 0x00;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_500DPS:
        gScaleFactor = 65.5;  // 32768 (max signed 4 bytes value) // 500
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_1000DPS:
        gScaleFactor = 32.8;  // 32768 (max signed 4 bytes value) // 1000
        select = 0x10;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_2000DPS:
        gScaleFactor = 16.4; // 32768 (max signed 4 bytes value) // 2000
        select = 0x18;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        gScaleFactor = 65.5;  // 32768 (max signed 4 bytes value) // 500
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }
}

/// @brief Read raw data from IMU.
/// @param I2Cx Pointer to I2C structure config.
void MPU_readRawData(I2C_HandleTypeDef *I2Cx)
{
    // Init buffer
    uint8_t buf[14];

    // Subroutine for reading the raw data
    HAL_I2C_Mem_Read(I2Cx, _addr, ACCEL_XOUT_H, 1, buf, 14, I2C_TIMOUT_MS);

    // Bit shift the data
    // accelerometer
    rawData.ax = buf[0] << 8 | buf[1];
    rawData.ay = buf[2] << 8 | buf[3];
    rawData.az = buf[4] << 8 | buf[5];
    
    // gyroscope
    rawData.gx = buf[8] << 8 | buf[9];
    rawData.gy = buf[10] << 8 | buf[11];
    rawData.gz = buf[12] << 8 | buf[13];

    // magnetometer
    uint8_t readData; // data ready status
	HAL_I2C_Mem_Read(I2Cx, MAG_ADDRESS, MAG_ST1, 1, &readData, 1, I2C_TIMOUT_MS);
	if( readData & 0x01 ){
		// uint8_t buf_mag[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
		// HAL_I2C_Mem_Read(I2Cx, MAG_ADDRESS, MAG_HXL, 1, &buf_mag[0], 7, I2C_TIMOUT_MS);  // Read raw magnetometer data and overflow status byte
		// uint8_t c = buf_mag[6];
		// if(!(c & 0x08)) {
		// 	rawData.mx = ((int16_t)buf_mag[1] << 8) | buf_mag[0];  // Concat to monolith value
		// 	rawData.my = ((int16_t)buf_mag[3] << 8) | buf_mag[2];   
		// 	rawData.mz = ((int16_t)buf_mag[5] << 8) | buf_mag[4];
	    // }

        uint8_t buf_mag[6];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
		HAL_I2C_Mem_Read(I2Cx, MAG_ADDRESS, MAG_HXL, 1, &buf_mag[0], 6, I2C_TIMOUT_MS);  // Read raw magnetometer data and overflow status byte

        rawData.mx = ((int16_t)buf_mag[1] << 8) | buf_mag[0];  // Concat to monolith value
        rawData.my = ((int16_t)buf_mag[3] << 8) | buf_mag[2];   
        rawData.mz = ((int16_t)buf_mag[5] << 8) | buf_mag[4];
    }

    // termometer
    rawData.t = buf[6] << 8 | buf[7]; 
}

/// @brief Find offsets for each axis of gyroscope.
/// @param I2Cx Pointer to I2C structure config.
/// @param numCalPoints Number of data points to average.
void MPU_calibrateGyro(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints)
{
    // Init
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;

    // Zero guard
    if (numCalPoints == 0)
    {
        numCalPoints = 1;
    }

    // Save specified number of points
    for (uint16_t ii = 0; ii < numCalPoints; ii++)
    {
        MPU_readRawData(I2Cx);
        x += rawData.gx;
        y += rawData.gy;
        z += rawData.gz;
        HAL_Delay(3);
    }

    // Average the saved data points to find the gyroscope offset
    gyroCal.x = (float)x / (float)numCalPoints;
    gyroCal.y = (float)y / (float)numCalPoints;
    gyroCal.z = (float)z / (float)numCalPoints;
}

/// @brief Calculate the real world sensor values.
/// @param I2Cx Pointer to I2C structure config.
void MPU_readProcessedData(I2C_HandleTypeDef *I2Cx)
{
    // Get raw values from the IMU
    MPU_readRawData(I2Cx);

    // Convert accelerometer values to g's
    sensorData.ax = rawData.ax / aScaleFactor;
    sensorData.ay = rawData.ay / aScaleFactor;
    sensorData.az = rawData.az / aScaleFactor;
    

    // Compensate for gyro offset
    sensorData.gx = rawData.gx - gyroCal.x;
    sensorData.gy = rawData.gy - gyroCal.y;
    sensorData.gz = rawData.gz - gyroCal.z;

    // Convert gyro values to deg/s
    sensorData.gx /= gScaleFactor;
    sensorData.gy /= gScaleFactor;
    sensorData.gz /= gScaleFactor;

    // Convert mag values to uT
    sensorData.mx /= mScaleXFactor;
	sensorData.my /= mScaleYFactor; 
	sensorData.mz /= mScaleZFactor;

    // Convert termometer values to celsius
    sensorData.t  = ((float)rawData.t - 21.0f) / 333.87f + 21.0f; //rawData.t / (float)340.0 + (float)36.53;
}

/// @brief Calculate the attitude of the sensor in degrees using a complementary filter.
/// @param I2Cx Pointer to I2C structure config.
void MPU_calcAttitude(I2C_HandleTypeDef *I2Cx)
{
    // Read processed data
    MPU_readProcessedData(I2Cx);

    // Complementary filter
    float accelPitch = atan2(sensorData.ay, sensorData.az) * RAD2DEG;
    float accelRoll = atan2(sensorData.ax, sensorData.az) * RAD2DEG;

    attitude.r = tau * (attitude.r - sensorData.gy * dt) + (1 - tau) * accelRoll;
    attitude.p = tau * (attitude.p + sensorData.gx * dt) + (1 - tau) * accelPitch;
    attitude.y += sensorData.gz * dt;
}
