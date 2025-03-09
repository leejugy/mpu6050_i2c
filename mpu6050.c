#include "mpu6050.h"

static int read_mpu6050_buffer(MPU6050_REG reg_addr, uint8_t *buffer, size_t buffer_size)
{
    return i2c_ioctl_read_8bit(I2C_MPU6050, reg_addr, buffer, buffer_size);
}

static int write_mpu6050_buffer(MPU6050_REG reg_addr, uint8_t *buffer, size_t buffer_size)
{
    return i2c_ioctl_write_8bit(I2C_MPU6050, reg_addr, buffer, buffer_size);
}

static void get_mpu6050_power_mode1()
{
    uint8_t mpu6050_buffer = 0;
    read_mpu6050_buffer(MPU6050_PWR_MGMT_1, &mpu6050_buffer, sizeof(mpu6050_buffer));
    DEBUG("pwr_mgmt_1 : 0x%02x", mpu6050_buffer);
}

static void set_mpu6050_wake_up()
{
    uint8_t mpu6050_buffer = 0x00; //sleep bit set 0, and cycle bit set 1!
    write_mpu6050_buffer(MPU6050_PWR_MGMT_1, &mpu6050_buffer, sizeof(mpu6050_buffer));
    DEBUG("pwr_mgmt_1 : 0x%02x", mpu6050_buffer);
}

static void get_mpu6050_temperature()
{
    uint8_t mpu6050_buffer[BURST_READ_LEN] = {0, };
    read_mpu6050_buffer(MPU6050_TEMP_OUT_H, mpu6050_buffer, sizeof(mpu6050_buffer));
    DEBUG("temperature : 0x%02x 0x%02x", mpu6050_buffer[0], mpu6050_buffer[1]);
}

static void get_mpu6050_addr()
{
    uint8_t mpu6050_buffer = 0;
    read_mpu6050_buffer(MPU6050_WHO_AM_I, &mpu6050_buffer, sizeof(mpu6050_buffer));
    DEBUG("who am i : 0x%02x", mpu6050_buffer);
}

static void thread_mpu6050()
{
    check_valid_i2c_address_8bit_ioctl(I2C_MPU6050);
    set_mpu6050_wake_up();

    while(1)
    {
        get_mpu6050_power_mode1();
        get_mpu6050_temperature();
        get_mpu6050_addr();
        sleep(1);
    }
}

static int init_mpu6050()
{
    int ret = 0;

    ret = init_i2c_dev(I2C1, I2C_MPU6050);
    if(ret < 0)
    {
        return ret;
    }
    
    return ret;
}

void start_mpu6050_thread()
{
    pthread_t tid = 0;
    int ret = 0;

    ret = init_mpu6050();
    if(ret < 0)
    {
        FATAL("fail to init i2c");
    }

    ret = pthread_create(&tid, NULL, (void *)&thread_mpu6050, NULL);
    if(ret < 0)
    {
        perror("fail to open mpu6050 thread");
        return;
    }

    ret = pthread_detach(tid);
    if(ret < 0)
    {
        perror("fail to detach");
        return;
    }
}