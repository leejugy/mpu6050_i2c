#include "mpu6050.h"

int bit_resolution = 0;

static int16_t get_burst_signed_reg_value(uint8_t *buffer)
{
    return (buffer[0] << 8) + buffer[1];
}

static uint16_t get_burst_unsigned_reg_value(uint8_t *buffer)
{
    return (buffer[0] << 8) + buffer[1];
}

static int read_mpu6050_buffer(MPU6050_REG reg_addr, uint8_t *buffer, size_t buffer_size)
{
    return i2c_ioctl_read_8bit(I2C_MPU6050, reg_addr, buffer, buffer_size);
}

static int write_mpu6050_buffer(MPU6050_REG reg_addr, uint8_t *buffer, size_t buffer_size)
{
    return i2c_ioctl_write_8bit(I2C_MPU6050, reg_addr, buffer, buffer_size);
}

static int16_t get_mpu6050_singed_burst_read(MPU6050_REG reg_addr)
{
    uint8_t mpu6050_buffer[BURST_READ_LEN] = {
        0,
    };

    read_mpu6050_buffer(reg_addr, mpu6050_buffer, sizeof(mpu6050_buffer));
    return get_burst_signed_reg_value(mpu6050_buffer);
}

static int16_t get_mpu6050_unsinged_burst_read(MPU6050_REG reg_addr)
{
    uint8_t mpu6050_buffer[BURST_READ_LEN] = {
        0,
    };

    read_mpu6050_buffer(reg_addr, mpu6050_buffer, sizeof(mpu6050_buffer));
    return get_burst_unsigned_reg_value(mpu6050_buffer);
}

static void get_mpu6050_power_mode1()
{
    uint8_t mpu6050_buffer = 0;
    read_mpu6050_buffer(MPU6050_PWR_MGMT_1, &mpu6050_buffer, sizeof(mpu6050_buffer));
    DEBUG("pwr_mgmt_1 : 0x%02x", mpu6050_buffer);
}

static void set_mpu6050_wake_up()
{
    uint8_t mpu6050_buffer = 0x00; // sleep bit set 0, and cycle bit set 1!
    write_mpu6050_buffer(MPU6050_PWR_MGMT_1, &mpu6050_buffer, sizeof(mpu6050_buffer));
    DEBUG("pwr_mgmt_1 : 0x%02x", mpu6050_buffer);
}

static void set_mpu6050_accelerator_resolution(MPU6050_AFS_SEL range)
{
    mpu6050_accelerometer_config_reg config;
    memset(&config, 0, sizeof(config));

    switch (range)
    {
    case MPU6050_SCALE_2G:
        bit_resolution = ACC_RESOLUTION_2G;
        break;

    case MPU6050_SCALE_4G:
        bit_resolution = ACC_RESOLUTION_4G;
        break;

    case MPU6050_SCALE_8G:
        bit_resolution = ACC_RESOLUTION_8G;
        break;

    case MPU6050_SCALE_16G:
        bit_resolution = ACC_RESOLUTION_16G;
        break;

    default:
        break;
    }

    config.x_acc_selftest = MPU6050_OFF_TEST;
    config.y_acc_selftest = MPU6050_OFF_TEST;
    config.z_acc_selftest = MPU6050_OFF_TEST;
    config.acc_resoultion = range;

    write_mpu6050_buffer(MPU6050_ACCEL_CONFIG, (uint8_t *)&config, sizeof(config));
}

static void get_mpu6050_accelerator_resolution()
{
    uint8_t mpu6050_buffer = 0;
    read_mpu6050_buffer(MPU6050_ACCEL_CONFIG, &mpu6050_buffer, sizeof(mpu6050_buffer));
    DEBUG("accelerometer config : 0x%02x", mpu6050_buffer);
}

static void get_mpu6050_x_accelerometer()
{
    int16_t x_acc = get_mpu6050_singed_burst_read(MPU6050_ACCEL_XOUT_H);
    DEBUG("x accelerometer : %05.02f[G]", MPU6050_CALCULATE_ACC(x_acc, bit_resolution));
}

static void get_mpu6050_y_accelerometer()
{
    int16_t y_acc = get_mpu6050_singed_burst_read(MPU6050_ACCEL_YOUT_H);
    DEBUG("y accelerometer : %05.02f[G]", MPU6050_CALCULATE_ACC(y_acc, bit_resolution));
}

static void get_mpu6050_z_accelerometer()
{
    int16_t z_acc = get_mpu6050_singed_burst_read(MPU6050_ACCEL_ZOUT_H);
    DEBUG("z accelerometer : %05.02f[G]", MPU6050_CALCULATE_ACC(z_acc, bit_resolution));
}

static void get_mpu6050_temperature()
{
    int16_t temperature = get_mpu6050_singed_burst_read(MPU6050_TEMP_OUT_H);
    DEBUG("temperature : %05.02f[c]", MPU6050_CALCULATE_TEMP(temperature));
}

static void get_mpu6050_addr()
{
    uint8_t mpu6050_buffer = 0;
    read_mpu6050_buffer(MPU6050_WHO_AM_I, &mpu6050_buffer, sizeof(mpu6050_buffer));
    DEBUG("who am i : 0x%02x", mpu6050_buffer);
}

static void thread_mpu6050()
{
    while (1)
    {
        get_mpu6050_temperature();
        get_mpu6050_x_accelerometer();
        get_mpu6050_y_accelerometer();
        get_mpu6050_z_accelerometer();
        sleep(1);
    }
}

static void set_config_mpu6050()
{
    set_mpu6050_wake_up();
    set_mpu6050_accelerator_resolution(MPU6050_SCALE_16G);
    get_mpu6050_addr();
    get_mpu6050_power_mode1();
    get_mpu6050_accelerator_resolution();
}

static int init_mpu6050()
{
    int ret = 0;

    ret = init_i2c_dev(I2C1, I2C_MPU6050);
    if (ret < 0)
    {
        return ret;
    }

    ret = check_valid_i2c_address_8bit_ioctl(I2C_MPU6050);
    if (ret < 0)
    {
        return ret;
    }

    set_config_mpu6050();
    return ret;
}

void start_mpu6050_thread()
{
    pthread_t tid = 0;
    int ret = 0;

    ret = init_mpu6050();
    if (ret < 0)
    {
        FATAL("fail to init i2c");
    }

    ret = pthread_create(&tid, NULL, (void *)&thread_mpu6050, NULL);
    if (ret < 0)
    {
        perror("fail to open mpu6050 thread");
        return;
    }

    ret = pthread_detach(tid);
    if (ret < 0)
    {
        perror("fail to detach");
        return;
    }
}