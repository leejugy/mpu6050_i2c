#ifndef __I2C__
#define __I2C__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <semaphore.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define MPU6050_ADDRESS 0x68
#define I2C0_ROUTE "/dev/i2c-0"
#define I2C1_ROUTE "/dev/i2c-1"
#define I2C2_ROUTE "/dev/i2c-2"

#define FATAL(fmt, ...) printf("[\x1b[31m%s\x1b[0m]" fmt "\n", __FUNCTION__, ##__VA_ARGS__)
#define DEBUG(fmt, ...) printf("[\x1b[35m%s\x1b[0m]" fmt "\n", __FUNCTION__, ##__VA_ARGS__)

typedef enum
{
    I2C_REG,
    I2C_DATA,
    I2C_8BIT_MSG_NUM,
}I2C_DATA_INDEX;

typedef enum
{
    I2C0,
    I2C1,
    I2C2,
    I2C_MAX_INDEX,
}I2C_INDEX;

typedef enum
{
    I2C_MPU6050,
    I2C_MAX_DEVICE,
}I2C_DEVICE;

typedef struct
{
    int fd;
    sem_t sem;
}i2c_struct_t;

#endif