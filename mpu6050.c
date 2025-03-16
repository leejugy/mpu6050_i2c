/**
 * @ref 참고로 burst_read는 동일 셈플링레이트로 변환된 데이터 2바이트를 읽어오는 것을 의미한다.
 */
#include "mpu6050.h"

int bit_resolution = 0;

/**
 * @brief 부호 있는 2바이트 데이터를 변환한다.
 * 
 * @param buffer 2바이트 읽은 버퍼
 * @return int16_t 부호 있는 2바이트 데이터를 반환
 */
static int16_t get_burst_signed_reg_value(uint8_t *buffer)
{
    return (buffer[0] << 8) + buffer[1];
}

/**
 * @brief 부호 없는 2바이트 데이터를 변환한다.
 * 
 * @param buffer 2바이트 읽은 버퍼
 * @return int16_t 부호 없는 2바이트 데이터를 반환
 */
static uint16_t get_burst_unsigned_reg_value(uint8_t *buffer)
{
    return (buffer[0] << 8) + buffer[1];
}

/**
 * @brief mpu6050 디바이스를 읽어 온다.
 * @note 만약 unistd를 시험해 보고 싶으면 아래 ioctl 부분을 -> unistd로 변경
 * 
 * @param reg_addr 레지스터 값
 * @param buffer 버퍼
 * @param buffer_size 버퍼 크기
 * @return int 실패시 -1, 성공시 1
 */
static int read_mpu6050_buffer(MPU6050_REG reg_addr, uint8_t *buffer, size_t buffer_size)
{
    return i2c_ioctl_read_8bit(I2C_MPU6050, reg_addr, buffer, buffer_size);
}

/**
 * @brief mpu6050 디바이스를 쓴다.
 * 
 * @param reg_addr 레지스터 값
 * @param buffer 버퍼
 * @param buffer_size 버퍼 크기
 * @return int 실패시 -1, 성공시 1
 */
static int write_mpu6050_buffer(MPU6050_REG reg_addr, uint8_t *buffer, size_t buffer_size)
{
    return i2c_ioctl_write_8bit(I2C_MPU6050, reg_addr, buffer, buffer_size);
}

/**
 * @brief 부호 있는 mpu6050 레지스터 값을 읽어 와서 반환한다. burst read이다.
 * 
 * @param reg_addr 레지스터 주소
 * @return int16_t 읽어낸 레지스터의 값
 */
static int16_t get_mpu6050_singed_burst_read(MPU6050_REG reg_addr)
{
    uint8_t mpu6050_buffer[BURST_READ_LEN] = {
        0,
    };
    int ret = 0;

    ret = read_mpu6050_buffer(reg_addr, mpu6050_buffer, sizeof(mpu6050_buffer));
    if(ret < 0)
    {
        return ret;
    }

    return get_burst_signed_reg_value(mpu6050_buffer);
}

/**
 * @brief 부호 없는 mpu6050 레지스터 값을 읽어 와서 반환한다. burst read이다.
 * 
 * @param reg_addr 레지스터 주소
 * @return int16_t 읽어낸 레지스터의 값
 */
static int16_t get_mpu6050_unsinged_burst_read(MPU6050_REG reg_addr)
{
    uint8_t mpu6050_buffer[BURST_READ_LEN] = {
        0,
    };
    int ret = 0;

    ret = read_mpu6050_buffer(reg_addr, mpu6050_buffer, sizeof(mpu6050_buffer));
    if(ret < 0)
    {
        return ret;
    }

    return get_burst_unsigned_reg_value(mpu6050_buffer);
}

/**
 * @brief mpu 파워 모드 1을 가져온다.
 * 
 */
static void get_mpu6050_power_mode1()
{
    uint8_t mpu6050_buffer = 0;
    int ret = 0;
    ret = read_mpu6050_buffer(MPU6050_PWR_MGMT_1, &mpu6050_buffer, sizeof(mpu6050_buffer));
    if(ret < 0)
    {
        FATAL("fail to read power management 1");
        return;
    }

    DEBUG("pwr_mgmt_1 : 0x%02x", mpu6050_buffer);
}

/**
 * @brief mpu6050을 슬립 모드에서 해제한다.
 * 
 */
static void set_mpu6050_wake_up()
{
    uint8_t mpu6050_buffer = 0x00; // sleep bit set 0, and cycle bit set 1!
    int ret = 0;
    ret = write_mpu6050_buffer(MPU6050_PWR_MGMT_1, &mpu6050_buffer, sizeof(mpu6050_buffer));
    if(ret < 0)
    {
        FATAL("fail to write power management 1");
        return;
    }

    DEBUG("pwr_mgmt_1 : 0x%02x", mpu6050_buffer);
}

/**
 * @brief mpu6050의 가속도 범주를 설정한다.
 * 
 * @param range 가속도 범주
 */
static void set_mpu6050_accelerator_resolution(MPU6050_AFS_SEL range)
{
    int ret = 0;
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

    ret = write_mpu6050_buffer(MPU6050_ACCEL_CONFIG, (uint8_t *)&config, sizeof(config));
    if(ret < 0)
    {
        FATAL("fail to write acc config");
    }
}

/**
 * @brief mpu6050의 가속도 범주를 가져온다.
 * 
 */
static void get_mpu6050_accelerator_resolution()
{
    uint8_t mpu6050_buffer = 0;
    int ret = 0;
    ret = read_mpu6050_buffer(MPU6050_ACCEL_CONFIG, &mpu6050_buffer, sizeof(mpu6050_buffer));
    if(ret < 0)
    {
        FATAL("fail to read acc config");
        return;
    }

    DEBUG("accelerometer config : 0x%02x", mpu6050_buffer);
}

/**
 * @brief mpu6050의 x 가속도 값을 가져온다.
 * 
 */
static void get_mpu6050_x_accelerometer()
{
    int16_t x_acc = get_mpu6050_singed_burst_read(MPU6050_ACCEL_XOUT_H);
    if(x_acc < 0)
    {
        FATAL("fail to read x acc");
        return;
    }

    INFO("x accelerometer : %05.02f[G]", MPU6050_CALCULATE_ACC(x_acc, bit_resolution));
}

/**
 * @brief mpu6050의 y 가속도 값을 가져온다.
 * 
 */
static void get_mpu6050_y_accelerometer()
{
    int16_t y_acc = get_mpu6050_singed_burst_read(MPU6050_ACCEL_YOUT_H);
    if(y_acc < 0)
    {
        FATAL("fail to read y acc");
        return;
    }

    INFO("y accelerometer : %05.02f[G]", MPU6050_CALCULATE_ACC(y_acc, bit_resolution));
}

/**
 * @brief mpu6050의 z 가속도 값을 가져온다.
 * 
 */
static void get_mpu6050_z_accelerometer()
{
    int16_t z_acc = get_mpu6050_singed_burst_read(MPU6050_ACCEL_ZOUT_H);
    if(z_acc < 0)
    {
        FATAL("fail to read z acc");
        return;
    }

    INFO("z accelerometer : %05.02f[G]", MPU6050_CALCULATE_ACC(z_acc, bit_resolution));
}

/**
 * @brief mpu6050의 온도 값을 가져온다.
 * 
 */
static void get_mpu6050_temperature()
{
    int16_t temperature = get_mpu6050_singed_burst_read(MPU6050_TEMP_OUT_H);
    if(temperature < 0)
    {
        FATAL("fail to read temperature");
        return;
    }

    INFO("temperature : %05.02f[c]", MPU6050_CALCULATE_TEMP(temperature));
}

/**
 * @brief mpu6050의 주소 값을 가져온다.
 * 
 */
static void get_mpu6050_addr()
{
    uint8_t mpu6050_buffer = 0;
    int ret = 0;

    ret = read_mpu6050_buffer(MPU6050_WHO_AM_I, &mpu6050_buffer, sizeof(mpu6050_buffer));
    if(ret < 0)
    {
        FATAL("fail to read who am i");
    }

    DEBUG("who am i : 0x%02x", mpu6050_buffer);
}

//스레드
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

//설정
static void set_config_mpu6050()
{
    set_mpu6050_wake_up();
    set_mpu6050_accelerator_resolution(MPU6050_SCALE_16G);
    get_mpu6050_addr();
    get_mpu6050_power_mode1();
    get_mpu6050_accelerator_resolution();
}

//mpu6050 초기화
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