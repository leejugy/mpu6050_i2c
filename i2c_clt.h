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
} I2C_DATA_INDEX;

typedef enum
{
    I2C0,
    I2C1,
    I2C2,
    I2C_MAX_INDEX,
} I2C_INDEX;

typedef enum
{
    I2C_MPU6050,
    I2C_MAX_DEVICE,
} I2C_DEVICE;

typedef struct
{
    int fd;
    uint16_t address;
    sem_t *sem;
} i2c_struct_t;

typedef struct
{
    int fd;
    sem_t sem;
} i2c_driver_t;

//!ioctl 함수던 read/write 함수로 레지스터에 쓰려고 하면, 연속적으로 보내야함. 안그러면 개별 데이터로 인식

int init_i2c();
int init_i2c_dev(I2C_INDEX i2c_index, I2C_DEVICE i2c_dev);
int set_i2c_address(I2C_DEVICE i2c_dev); //std 보내기 전에 설정하고 보낼 것

/********************************/
/* ioctl */
/********************************/
int i2c_ioctl_read_8bit(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size);
int i2c_ioctl_write_8bit(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size);
int check_valid_i2c_address_8bit_ioctl(I2C_DEVICE i2c_dev);


/********************************/
/* unistd */
/********************************/
int i2c_unistd_read_8bit(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size);
int i2c_unistd_write_8bit(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size);
int check_valid_i2c_address_8bit_unistd(I2C_DEVICE i2c_dev);

#endif