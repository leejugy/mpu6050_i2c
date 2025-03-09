#ifndef __MPU6050__
#define __MPU6050__

#include "i2c_clt.h"
#include <pthread.h>

void start_mpu6050_thread();

#define GET_1BIT(val, bit_num) (val >> bit_num) & 0x01
#define BURST_READ_LEN 2

typedef enum
{
    MPU6050_TEMP_OUT_H = 65,
    MPU6050_TEMP_OUT_L = 66,
    MPU6050_PWR_MGMT_1 = 107,
    MPU6050_WHO_AM_I = 117,
}MPU6050_REG;

#endif