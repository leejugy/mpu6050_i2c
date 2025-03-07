#include "i2c_clt.h"

i2c_struct_t i2c_st[I2C_MAX_DEVICE] = {0, };

static int get_i2c_address(I2C_DEVICE i2c_dev)
{
    int ret = -1;

    switch (i2c_dev)
    {
    case I2C_MPU6050:
        ret = MPU6050_ADDRESS;
        break;
    
    default:
        break;
    }

    return ret;
}

static int set_i2c_address(I2C_DEVICE i2c_dev)
{
    int address = 0;
    int ret = 0;

    address = get_i2c_address(i2c_dev);
    if(address < 0)
    {
        FATAL("fail to get address unkown, i2c_dev : %d", i2c_dev);
        close(i2c_st[i2c_dev].fd);
        return -1;
    }

    ret = ioctl(i2c_st[i2c_dev].fd, I2C_SLAVE, address);
    if(ret < 0)
    {
        FATAL("ioctl fail i2c_dev : %d", i2c_dev);
        close(i2c_st[i2c_dev].fd);
        return -1;
    }
}

static int init_i2c(I2C_INDEX i2c_index, I2C_DEVICE i2c_dev)
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

    ret = open(i2c_route, O_RDWR);
    if(ret < 0)
    {
        FATAL("fail to open i2c driver, i2c_route : %s", i2c_route);
        perror("error");
        return -1;
    }

    i2c_st[i2c_dev].fd = ret;

    ret = sem_init(&i2c_st[i2c_dev].sem, 0, 1);
    if(ret < 0)
    {
        FATAL("fail to init sem, i2c_dev : %d", i2c_dev);
        perror("error");
        close(i2c_st[i2c_dev].fd);
        return -1;
    }

    return 1;
}

int i2c_ioctl_read_8bit(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size)
{
    int ret = 0;
    struct i2c_msg i2c_dev_msg[I2C_8BIT_MSG_NUM] = {0, };
    struct i2c_rdwr_ioctl_data i2c_buf = {0, };

    i2c_dev_msg[I2C_REG].addr = MPU6050_ADDRESS;
    i2c_dev_msg[I2C_REG].buf = &reg_address;
    i2c_dev_msg[I2C_REG].flags = 0; //write mode
    i2c_dev_msg[I2C_REG].len = sizeof(reg_address);

    i2c_dev_msg[I2C_DATA].addr = MPU6050_ADDRESS;
    i2c_dev_msg[I2C_DATA].buf = buffer;
    i2c_dev_msg[I2C_DATA].flags = I2C_M_RD; //read mode
    i2c_dev_msg[I2C_DATA].len = buffer_size;

    i2c_buf.msgs = i2c_dev_msg;
    i2c_buf.nmsgs = sizeof(i2c_dev_msg)/sizeof(struct i2c_msg);

    sem_wait(&i2c_st[i2c_dev].sem);
    ret = ioctl(i2c_st[i2c_dev].fd, I2C_RDWR, &i2c_buf);
    sem_post(&i2c_st[i2c_dev].sem);

    if(ret < 0)
    {
        FATAL("fail to read, slave : %d, reg : %d", MPU6050_ADDRESS, reg_address);
        return ret;
    }

    return 1;
}

int i2c_ioctl_write_8bit(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size)
{
    int ret = 0;
    struct i2c_msg i2c_dev_msg[I2C_8BIT_MSG_NUM] = {0, };
    struct i2c_rdwr_ioctl_data i2c_buf = {0, };

    i2c_dev_msg[I2C_REG].addr = MPU6050_ADDRESS;
    i2c_dev_msg[I2C_REG].buf = &reg_address;
    i2c_dev_msg[I2C_REG].flags = 0; //write mode
    i2c_dev_msg[I2C_REG].len = sizeof(reg_address);

    i2c_dev_msg[I2C_DATA].addr = MPU6050_ADDRESS;
    i2c_dev_msg[I2C_DATA].buf = buffer;
    i2c_dev_msg[I2C_DATA].flags = 0; //read mode
    i2c_dev_msg[I2C_DATA].len = buffer_size;

    i2c_buf.msgs = i2c_dev_msg;
    i2c_buf.nmsgs = sizeof(i2c_dev_msg)/sizeof(struct i2c_msg);

    sem_wait(&i2c_st[i2c_dev].sem);
    ret = ioctl(i2c_st[i2c_dev].fd, I2C_RDWR, &i2c_buf);
    sem_post(&i2c_st[i2c_dev].sem);
    
    if(ret < 0)
    {
        FATAL("fail to write, slave : %d, reg : %d", MPU6050_ADDRESS, reg_address);
        return ret;
    }

    return 1;
}

static int i2c_unistd_read_8bit_function(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size)
{
    int ret = 0;

    ret = write(i2c_st[i2c_dev].fd, &reg_address, sizeof(reg_address));
    if(ret < 0)
    {
        FATAL("fail to write reg address, slave : %d, reg : %d", MPU6050_ADDRESS, reg_address);
        return ret;
    }

    ret = read(i2c_st[i2c_dev].fd, buffer, buffer_size);
    if(ret < 0)
    {
        FATAL("fail to read, slave : %d, reg : %d", MPU6050_ADDRESS, reg_address);
        return ret;
    }

    return ret;
}

static int i2c_unistd_write_8bit_function(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size)
{
    int ret = 0;
    int len = 0;

    len = buffer_size + sizeof(reg_address);
    uint8_t *write_buf = calloc(sizeof(uint8_t), len);

    ret = write(i2c_st[i2c_dev].fd, write_buf, len);

    free(write_buf);

    if(ret < 0)
    {
        FATAL("fail to write, slave : %d, reg : %d", MPU6050_ADDRESS, reg_address);
        return ret;
    }

    return ret;
}

int i2c_unistd_read_8bit(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size)
{
    int ret = 0;

    sem_wait(&i2c_st[i2c_dev].sem);
    ret = i2c_unistd_read_8bit_function(i2c_dev, reg_address, buffer, buffer_size);
    sem_post(&i2c_st[i2c_dev].sem);

    return ret;
}

int i2c_unistd_write_8bit(I2C_DEVICE i2c_dev, uint8_t reg_address, void *buffer, size_t buffer_size)
{
    int ret = 0;

    sem_wait(&i2c_st[i2c_dev].sem);
    ret = i2c_unistd_write_8bit_function(i2c_dev, reg_address, buffer, buffer_size);
    sem_post(&i2c_st[i2c_dev].sem);

    return ret;
}