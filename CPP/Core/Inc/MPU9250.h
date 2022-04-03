/*
 * MPU9250.h
 *
 *  Created on: Apr 2, 2022
 *      Author: MarkSherstan
 */

#ifndef SRC_MPU9250_H_
#define SRC_MPU9250_H_

// Libraries
#include <stdint.h>
#include <math.h>
#include "SPI.h"

// Constants
#define RAD2DEG 57.2957795131

// Defines
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define WHO_AM_I 0x75
#define WHO_AM_I_9250_ANS 0x71

#define READWRITE 0x80
#define CS_SELECT 0
#define CS_DESELECT 1
#define SPI_TIMOUT_MS 1000

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

class MPU9250
{
private:
    // Functions
    // void REG_READ(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t addr, uint8_t *pRxData, uint16_t RxSize);
    // void REG_WRITE(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t *pAddr, uint8_t *pVal);
    // void setGyroFullScaleRange(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t gScale);
    // void setAccFullScaleRange(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t aScale);
    // void CS(MPU9250_t *pMPU9250, uint8_t state);

    // Variables
    SPI_HandleTypeDef *_pSPI;
    uint8_t _aFSR, _gFSR;

public:
    // Init
    MPU9250(SPI_HandleTypeDef *pSPI, uint8_t gFSR, uint8_t aFSR);

    // Functions
    uint8_t begin();
    // void alibrateGyro(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint16_t numCalPoints);
    // void readProcessedData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);
    // void calcAttitude(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);
    // void readRawData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);
};

#endif /* SRC_MPU9250_H_ */
