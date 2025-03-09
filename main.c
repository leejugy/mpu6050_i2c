#include "mpu6050.c"

static void init_drivers()
{
    int ret = 0;
    ret = init_i2c();

    if(ret < 0)
    {
        FATAL("fail to init i2c");
    }
}

int main()
{
    init_drivers();
    start_mpu6050_thread();
    while(1);
}