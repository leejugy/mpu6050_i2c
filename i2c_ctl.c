#include "i2c_clt.h"

i2c_struct_t i2c_device[I2C_MAX_DEVICE] = {
    0,
};
i2c_driver_t i2c_driver[I2C_MAX_INDEX] = {
    0,
};

/**
 * @brief i2c 디바이스의 슬레이브 주소를 반환.
 * 
 * @param i2c_dev i2c 디바이스 인덱스
 * @return int 디바이스 없으면 -1, 있으면 주소 반환
 */
static int get_i2c_address(I2C_DEVICE i2c_dev)
{
    int ret = -1;

    switch (i2c_dev)
    {
    case I2C_MPU6050:
        ret = MPU6050_ADDRESS; //mpu 6050 슬레이브 주소
        break;

    default:
        ret = -1;
        break;
    }

    return ret;
}

/**
 * @brief i2c 디바이스가 사용하는 버스의 슬레이브 주소를 설정한다.
 * 
 * @param i2c_dev 사용하는 i2c 디바이스
 * @return int 디바이스 슬레이브 주소 설정 성공시 1, 실패시 -1
 */
int set_i2c_address(I2C_DEVICE i2c_dev)
{
    int address = 0;
    int ret = 0;

    address = get_i2c_address(i2c_dev);
    if (address < 0)
    {
        FATAL("fail to get address unkown, i2c_dev : %d", i2c_dev);
        close(i2c_device[i2c_dev].fd);
        return -1;
    }

    ret = ioctl(i2c_device[i2c_dev].fd, I2C_SLAVE, address);
    if (ret < 0)
    {
        FATAL("ioctl fail i2c_dev : %d", i2c_dev);
        close(i2c_device[i2c_dev].fd);
        return -1;
    }
}

/**
 * @brief i2c-[버스 번호]를 열고 세마포어를 초기화 하는 함수다.
 * 
 * @param i2c_index i2c 인덱스 (0,1,2)
 * @return int 실패시 -1, 성공시 1
 */
static int init_i2c_driver(I2C_INDEX i2c_index)
{
    char *i2c_route = NULL;
    int ret = 0;

    switch (i2c_index)
    {
    case I2C0:
        i2c_route = I2C0_ROUTE;
        break;

    case I2C1:
        i2c_route = I2C1_ROUTE;
        break;

    case I2C2:
        i2c_route = I2C0_ROUTE;
        break;

    default:
        break;
    }

    ret = open(i2c_route, O_RDWR); //i2c-[버스 번호]를 읽기/쓰기 권한을 주어 오픈한다.
    if (ret < 0)
    {
        FATAL("fail to open i2c driver, i2c_route : %s", i2c_route);
        perror("error");
        return -1;
    }

    i2c_driver[i2c_index].fd = ret;

    ret = sem_init(&i2c_driver[i2c_index].sem, 0, 1);
    if (ret < 0)
    {
        FATAL("fail to init sem, i2c_dev : %d", i2c_index);
        perror("error");
        close(i2c_driver[i2c_index].fd);
        return -1;
    }

    return 1;
}

int init_i2c_dev(I2C_INDEX i2c_index, I2C_DEVICE i2c_dev)
{
    int ret = 0;

    i2c_device[i2c_dev].fd = i2c_driver[i2c_index].fd;    // i2c 드라이버의 파일디스크립터를 받아 저장
    i2c_device[i2c_dev].sem = &i2c_driver[i2c_index].sem; // i2c 드라이버의 세마포어를 그대로 가져온다.

    ret = get_i2c_address(i2c_dev);
    if (ret < 0)
    {
        FATAL("unkown_device : %d", i2c_dev);
        close(i2c_device[i2c_dev].fd);
        return -1;
    }

    i2c_device[i2c_dev].address = ret;

    return 1;
}

int init_i2c()
{
    return init_i2c_driver(I2C1); // i2c1만 초기화 하자.
}

int i2c_ioctl_read_8bit(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size)
{
    int ret = 0;
    struct i2c_msg i2c_dev_msg[I2C_8BIT_MSG_NUM] = {
        0,
    }; //배열 크기를 2로 설정
    struct i2c_rdwr_ioctl_data i2c_buf = {
        0,
    };

    i2c_dev_msg[I2C_REG].addr = i2c_device[i2c_dev].address;
    i2c_dev_msg[I2C_REG].buf = &reg_address;
    i2c_dev_msg[I2C_REG].flags = 0; // write mode
    i2c_dev_msg[I2C_REG].len = sizeof(reg_address);

    i2c_dev_msg[I2C_DATA].addr = i2c_device[i2c_dev].address;
    i2c_dev_msg[I2C_DATA].buf = buffer;
    i2c_dev_msg[I2C_DATA].flags = I2C_M_RD; // read mode
    i2c_dev_msg[I2C_DATA].len = buffer_size;

    i2c_buf.msgs = i2c_dev_msg; //메시지 배열 주소
    i2c_buf.nmsgs = sizeof(i2c_dev_msg) / sizeof(struct i2c_msg); //메시지 크기

    sem_wait(i2c_device[i2c_dev].sem);
    ret = ioctl(i2c_device[i2c_dev].fd, I2C_RDWR, &i2c_buf); // ioctl 함수로 i2c 드라이버에 구조체 전송
    sem_post(i2c_device[i2c_dev].sem);

    if (ret < 0)
    {
        FATAL("fail to read, slave : %d, reg : %d", i2c_device[i2c_dev].address, reg_address);
        perror("ioctl read fail");
        return ret;
    }

    return 1;
}

int i2c_ioctl_write_8bit(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size)
{
    int ret = 0;
    int len = 0;
    int buf_index = 0;

    struct i2c_msg i2c_dev_msg = {
        0,
    };
    struct i2c_rdwr_ioctl_data i2c_buf = {
        0,
    };

    len = buffer_size + sizeof(reg_address);
    uint8_t *write_buf = calloc(sizeof(uint8_t), len); // 동적할당해 여러 비트를 한 메세지에 보내도록 하자. 끊어서 전송하면 개별 전송으로 인식된다.

    write_buf[buf_index++] = reg_address;
    memcpy(write_buf + buf_index, buffer, buffer_size);

    i2c_dev_msg.addr = i2c_device[i2c_dev].address;
    i2c_dev_msg.buf = write_buf;
    i2c_dev_msg.flags = 0; // write mode
    i2c_dev_msg.len = len;

    i2c_buf.msgs = &i2c_dev_msg; //메시지 주소
    i2c_buf.nmsgs = sizeof(i2c_dev_msg) / sizeof(struct i2c_msg); //메시지 크기

    sem_wait(i2c_device[i2c_dev].sem);
    ret = ioctl(i2c_device[i2c_dev].fd, I2C_RDWR, &i2c_buf); // ioctl 함수로 i2c 드라이버에 구조체 전송
    sem_post(i2c_device[i2c_dev].sem);

    free(write_buf);

    if (ret < 0)
    {
        FATAL("fail to write, slave : %d, reg : %d", i2c_device[i2c_dev].address, reg_address);
        perror("ioctl read fail");
        return ret;
    }

    return 1;
}

int check_valid_i2c_address_8bit_ioctl(I2C_DEVICE i2c_dev)
{
    int ret = 0;
    uint8_t temp_buffer = 0x00;

    ret = i2c_ioctl_read_8bit(i2c_dev, 0x00, &temp_buffer, sizeof(temp_buffer)); // 0번지 비트를 읽어낼 수 있는지만 검사한다.
    if (ret < 0)
    {
        FATAL("fail to read data");
        return ret;
    }

    DEBUG("valid address");
    return 1;
}

/**
 * @brief 세마포어가 없는 unistd i2c 읽기
 */
static int i2c_unistd_read_8bit_function(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size)
{
    int ret = 0;

    set_i2c_address(i2c_dev); // unistd 방식은 항상 슬레이브 주소 설정해야 한다.

    ret = write(i2c_device[i2c_dev].fd, &reg_address, sizeof(reg_address));
    if (ret < 0)
    {
        FATAL("fail to write reg address, slave : %d, reg : %d", i2c_device[i2c_dev].address, reg_address);
        return ret;
    }

    ret = read(i2c_device[i2c_dev].fd, buffer, buffer_size);
    if (ret < 0)
    {
        FATAL("fail to read, slave : %d, reg : %d", i2c_device[i2c_dev].address, reg_address);
        perror("ioctl read fail");
        return ret;
    }

    return ret;
}

/**
 * @brief 세마포어가 없는 unistd i2c 쓰기
 */
static int i2c_unistd_write_8bit_function(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size)
{
    int ret = 0;
    int len = 0;
    int buf_index = 0;

    len = buffer_size + sizeof(reg_address);
    uint8_t *write_buf = calloc(sizeof(uint8_t), len); // 동적할당해 write를 한번에 할 수 있게 하자. 끊어서 전송하면 개별 전송으로 인식된다.

    write_buf[buf_index++] = reg_address;
    memcpy(write_buf + buf_index, buffer, buffer_size);

    set_i2c_address(i2c_dev); // 슬레이브 주소 설정

    ret = write(i2c_device[i2c_dev].fd, write_buf, len);

    free(write_buf);

    if (ret < 0)
    {
        FATAL("fail to write, slave : %d, reg : %d", i2c_device[i2c_dev].address, reg_address);
        perror("ioctl read fail");
        return ret;
    }

    return ret;
}

int i2c_unistd_read_8bit(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size)
{
    int ret = 0;

    sem_wait(i2c_device[i2c_dev].sem); //세마포어
    ret = i2c_unistd_read_8bit_function(i2c_dev, reg_address, buffer, buffer_size);
    sem_post(i2c_device[i2c_dev].sem);

    return ret;
}

int i2c_unistd_write_8bit(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size)
{
    int ret = 0;

    sem_wait(i2c_device[i2c_dev].sem); //세마포어
    ret = i2c_unistd_write_8bit_function(i2c_dev, reg_address, buffer, buffer_size);
    sem_post(i2c_device[i2c_dev].sem);

    return ret;
}

int check_valid_i2c_address_8bit_unistd(I2C_DEVICE i2c_dev)
{
    int ret = 0;
    uint8_t temp_buffer = 0x00;

    ret = i2c_unistd_read_8bit(i2c_dev, 0x00, &temp_buffer, sizeof(temp_buffer)); // 0번지 비트를 읽어낼 수 있는지만 검사한다.
    if (ret < 0)
    {
        FATAL("fail to read data");
        return ret;
    }

    DEBUG("read data : 0x%02x", temp_buffer);
    return 1;
}