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
#include <string>

extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

// Gyroscope and accelerometer configuration
#define GYRO_RANGE 0
#define ACCEL_RANGE 0

#define TAU 0.05 // Complementary Filter constant
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
    #define GYRO_SENS 131.0 // +/- 250 °/s
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
    #define ACCEL_SENS 16384.0 // +/- 2g
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
    /**
     * @brief Constructor. Opens the I2C bus and initializes the MPU6050 sensor.
     * @param addr The I2C slave address of the MPU6050 (default 0x68).
     * @param busPath The path to the I2C device (e.g., "/dev/i2c-7").
     */
    explicit MPU6050(uint8_t addr = 0x68, const std::string& busPath = "/dev/i2c-7");

    /**
     * @brief Destructor. Closes the I2C file descriptor.
     */
    virtual ~MPU6050() { if (f_dev >= 0) close(f_dev); }

    /**
     * @brief Calculates calibration offsets for the sensor (gyro zero rate and accel 1G bias).
     * This function is blocking and MUST be called with the sensor stationary.
     * Starts the background Complementary Filter update thread upon completion.
     * @param samples The number of readings to average for calibration (default 1000).
     */
    void calibrate(int samples = 1000);

    /**
     * @brief Synchronously reads and returns the latest Accelerometer data (G's).
     * @param x Pointer to a float to store X-axis acceleration (G).
     * @param y Pointer to a float to store Y-axis acceleration (G).
     * @param z Pointer to a float to store Z-axis acceleration (G).
     */
    void getAccel(float *x, float *y, float *z);

    /**
     * @brief Synchronously reads and returns the latest Gyroscope data (degrees/sec).
     * @param roll Pointer to a float to store Roll rate (X-axis, deg/s).
     * @param pitch Pointer to a float to store Pitch rate (Y-axis, deg/s).
     * @param yaw Pointer to a float to store Yaw rate (Z-axis, deg/s).
     */
    void getGyro(float *roll, float *pitch, float *yaw);

    /**
     * @brief Synchronously reads and returns the latest raw Accelerometer counts (compensated).
     * @param x Pointer to a float to store raw X-axis acceleration count.
     * @param y Pointer to a float to store raw Y-axis acceleration count.
     * @param z Pointer to a float to store raw Z-axis acceleration count.
     */
    void getAccelRaw(float *x, float *y, float *z);

    /**
     * @brief Synchronously reads and returns the latest raw Gyroscope counts (compensated).
     * @param roll Pointer to a float to store raw Roll rate count.
     * @param pitch Pointer to a float to store raw Pitch rate count.
     * @param yaw Pointer to a float to store raw Yaw rate count.
     */
    void getGyroRaw(float *roll, float *pitch, float *yaw);

    /**
     * @brief Retrieves the latest filtered angle calculated by the background thread.
     * @param axis The angle axis: 0 for Roll, 1 for Pitch, 2 for Yaw.
     * @param result Pointer to a float to store the resulting angle (degrees).
     * @return 0 on success, -1 if the axis index is invalid.
     */
    int getAngle(int axis, float *result);

    /**
     * @brief Flag to enable Yaw calculation in the Complementary Filter (experimental).
     */
    bool calc_yaw;
};

#endif
