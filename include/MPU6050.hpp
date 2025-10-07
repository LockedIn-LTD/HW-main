#ifndef MPU6050_HPP
#define MPU6050_HPP

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <ctime>
#include <cmath>
#include <thread>

extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

// Gyroscope and accelerometer configuration
#define GYRO_RANGE 0
#define ACCEL_RANGE 0

#define TAU 0.05
#define RAD_TO_DEG 57.29577951308

#if GYRO_RANGE == 1
    #define GYRO_SENS 65.5
    #define GYRO_CONFIG 0b00001000
#elif GYRO_RANGE == 2
    #define GYRO_SENS 32.8
    #define GYRO_CONFIG 0b00010000
#elif GYRO_RANGE == 3
    #define GYRO_SENS 16.4
    #define GYRO_CONFIG 0b00011000
#else
    #define GYRO_SENS 131.0
    #define GYRO_CONFIG 0b00000000
#endif

#if ACCEL_RANGE == 1
    #define ACCEL_SENS 8192.0
    #define ACCEL_CONFIG 0b00001000
#elif ACCEL_RANGE == 2
    #define ACCEL_SENS 4096.0
    #define ACCEL_CONFIG 0b00010000
#elif ACCEL_RANGE == 3
    #define ACCEL_SENS 2048.0
    #define ACCEL_CONFIG 0b00011000
#else
    #define ACCEL_SENS 16384.0
    #define ACCEL_CONFIG 0b00000000
#endif

class MPU6050 {
private:
    void _update();
    void _readRaw();

    float _accel_angle[3];
    float _gyro_angle[3];
    float _angle[3];

    float ax, ay, az, gr, gp, gy;

    int MPU6050_addr;
    int f_dev; 

    float dt;
    struct timespec start, end;

    bool _first_run;
    bool _calibrated;

    float A_OFF_X, A_OFF_Y, A_OFF_Z;
    float G_OFF_X, G_OFF_Y, G_OFF_Z;

    void _startThread();

public:
    explicit MPU6050(int8_t addr = 0x68, const char* bus = "/dev/i2c-7");

    void calibrate(int samples = 1000);
    void getAccel(float *x, float *y, float *z);
    void getGyro(float *roll, float *pitch, float *yaw);
    void getAccelRaw(float *x, float *y, float *z);
    void getGyroRaw(float *roll, float *pitch, float *yaw);
    int getAngle(int axis, float *result);

    bool calc_yaw;
};

#endif
