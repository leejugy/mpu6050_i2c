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
#define INFO(fmt, ...) printf("[\x1b[32minfo\x1b[0m]" fmt "\n", ##__VA_ARGS__)

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

/**
 * @brief i2c 초기화 하는 함수
 * 
 * @return int 실패시 -1, 성공시 1
 */
int init_i2c();
/**
 * @brief i2c 디바이스 초기화 i2c_dev에 선언된 디바이스를 초기화한다.
 * 
 * @param i2c_index i2c 인덱스이다. i2c0, i2c1, ...
 * @param i2c_dev i2c 디바이스 이다. 여기서는 mpu6050 밖에 없다.
 * @return int 실패시 -1, 성공시 1
 */
int init_i2c_dev(I2C_INDEX i2c_index, I2C_DEVICE i2c_dev);
/**
 * @brief i2c 통신을 할 슬레이브 주소를 설정한다.
 * 
 * @param i2c_dev 통신할 디바이스
 * @return int 실패시 -1, 성공시 1
 */
int set_i2c_address(I2C_DEVICE i2c_dev);

/********************************/
/* ioctl 사용 */
/********************************/

/**
 * @brief 8비트 i2c를 읽는 함수이다.
 * 
 * @param i2c_dev i2c 디바이스
 * @param reg_address 디바이스의 레지스터 주소
 * @param buffer 읽을 버퍼
 * @param buffer_size 읽어낼 크기
 * @return int 실패시 -1, 성공시 1
 */
int i2c_ioctl_read_8bit(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size);

/**
 * @brief 8비트 i2c를 쓰는 함수이다.
 * 
 * @param i2c_dev i2c 디바이스
 * @param reg_address 디바이스의 레지스터 주소
 * @param buffer 보낼 버퍼
 * @param buffer_size 버퍼의 크기
 * @return int 실패시 -1, 성공시 1
 */
int i2c_ioctl_write_8bit(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size);

/**
 * @brief 유효한 주소인지 체크한다.
 * 
 * @param i2c_dev i2c 디바이스
 * @return int 실패시 -1, 성공시 1
 */
int check_valid_i2c_address_8bit_ioctl(I2C_DEVICE i2c_dev);


/********************************/
/* unistd 사용 */
/********************************/

/**
 * @brief 8비트 i2c를 읽는 함수이다.
 * 
 * @param i2c_dev i2c 디바이스
 * @param reg_address 디바이스의 레지스터 주소
 * @param buffer 읽을 버퍼
 * @param buffer_size 읽어낼 크기
 * @return int 실패시 -1, 성공시 1
 */
int i2c_unistd_read_8bit(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size);

/**
 * @brief 8비트 i2c를 쓰는 함수이다.
 * 
 * @param i2c_dev i2c 디바이스
 * @param reg_address 디바이스의 레지스터 주소
 * @param buffer 보낼 버퍼
 * @param buffer_size 버퍼의 크기
 * @return int 실패시 -1, 성공시 1
 */
int i2c_unistd_write_8bit(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size);

/**
 * @brief 유효한 주소인지 체크한다.
 * 
 * @param i2c_dev i2c 디바이스
 * @return int 실패시 -1, 성공시 1
 */
int check_valid_i2c_address_8bit_unistd(I2C_DEVICE i2c_dev);

#endif