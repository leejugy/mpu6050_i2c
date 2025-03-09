source /opt/gcc-beaglebone/environment-setup-armv7at2hf-neon-poky-linux-gnueabi

export SRC="i2c_ctl.c main.c"
export OBJECT="mpu6050"
export CFLAGS="-lrt -Wunused -lpthread"
export CPPFLAGS=""
export LDFLAGS=""

make -j

cp $OBJECT ~/share