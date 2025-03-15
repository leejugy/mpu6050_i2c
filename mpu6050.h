#ifndef __MPU6050__
#define __MPU6050__

#include "i2c_clt.h"
#include <pthread.h>

void start_mpu6050_thread();

#define GET_1BIT(val, bit_num) (val >> bit_num) & 0x01
#define MPU6050_CALCULATE_TEMP(val) ((float)val)/340.0 + 36.53
#define MPU6050_CALCULATE_ACC(val, resolution) (float)val / (float)resolution
#define BURST_READ_LEN 2

#define ACC_RESOLUTION_2G 16384
#define ACC_RESOLUTION_4G 8192
#define ACC_RESOLUTION_8G 4096
#define ACC_RESOLUTION_16G 2048

typedef enum
{
    MPU6050_ACCEL_CONFIG = 28,
    MPU6050_ACCEL_XOUT_H = 59,
    MPU6050_ACCEL_XOUT_L = 60,
    MPU6050_ACCEL_YOUT_H = 61,
    MPU6050_ACCEL_YOUT_L = 62,
    MPU6050_ACCEL_ZOUT_H = 63,
    MPU6050_ACCEL_ZOUT_L = 64,
    MPU6050_TEMP_OUT_H = 65,
    MPU6050_TEMP_OUT_L = 66,
    MPU6050_PWR_MGMT_1 = 107,
    MPU6050_WHO_AM_I = 117,
}MPU6050_REG;

typedef enum
{
    MPU6050_SCALE_2G,
    MPU6050_SCALE_4G,
    MPU6050_SCALE_8G,
    MPU6050_SCALE_16G,
}MPU6050_AFS_SEL;

typedef enum
{
    MPU6050_OFF_TEST = 0x00,
    MPU6050_ON_TEST = 0x01,
}MPU6050_TEST;

// arm 기반 cpu들이 리틀 엔디안임에 주의하자. 낮은 주소에 LSB가 높은 주소에 MSB가 저장된다.
// ex) 0x98 -> x가속도 = 1, y 가속도 = 0, z 가속도 = 0, 가속도 해상도 = 3
// 구조체는 맨위에 선언된 맴버가 가장 낮은 주소를 갖는다는 것에 주의하자.
typedef struct
{
    uint8_t preserved : 3;      // 사용x, 가장 낮은 주소 LSB
    uint8_t acc_resoultion : 2; // 가속도 해상도
    uint8_t z_acc_selftest : 1; // z 가속도
    uint8_t y_acc_selftest : 1; // y 가속도
    uint8_t x_acc_selftest : 1; // x 가속도, 가장 높은 주소 MSB
} mpu6050_accelerometer_config_reg;

#endif