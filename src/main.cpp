#include <ADS1115.hpp>
#include <MPU6050.hpp>
#include <MAX30102.hpp>
#include <iostream>
#include <cstdio>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#define I2C_BUS "/dev/i2c-7"

int main()
{
    int file = open(I2C_BUS, O_RDWR);
    if (file < 0)
    {
        printf("Failed to create file to start I2C.\n");
        return 0;
    }

    MAX30102 sensor(I2C_BUS);

    if (!sensor.begin())
    {
        std::cerr << "Failed to initialize MAX30102!" << std::endl;
        return 1;
    }

    sensor.startHeartRate();

    std::cout << "MAX30102 initialized. Reading data..." << std::endl;

    while (true)
    {
        uint32_t red, ir;
        sensor.getFIFO(red, ir);

        std::cout << "Red: " << red
                  << ", IR: " << ir
                  << ", BPM: " << sensor.getLatestBPM()
                  << ", Temp(C): " << sensor.readTemperature()
                  << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // MPU 6050, needs to be cleaned up and overhauled, works currently

    /*  MPU6050 imu(0x68);

        // Calibrate once while stationary
        imu.calibrate(1000);

        for (size_t i = 0; i < 200; i++)
        {
            float roll, pitch, yaw;

            // Get filtered angles from the complementary filter
            imu.getAngle(0, &roll);  // roll
            imu.getAngle(1, &pitch); // pitch
            imu.getAngle(2, &yaw);   // yaw (if enabled)

            std::cout << "Angle (°) -> Roll: " << roll
                      << ", Pitch: " << pitch
                      << ", Yaw: " << yaw << std::endl;

            // Optional: still print raw sensor data
            float ax, ay, az;
            float gr, gp, gy;
            imu.getAccel(&ax, &ay, &az);
            imu.getGyro(&gr, &gp, &gy);
            std::cout << "Accel (g): " << ax << ", " << ay << ", " << az << std::endl;
            std::cout << "Gyro (°/s): " << gr << ", " << gp << ", " << gy << std::endl;

            usleep(500000); // 0.5s delay
        }
    */

    // ADC READING BOTH
    /*
    ADS1115 adc(file);

    uint8_t channel = 0; // Start with A0
    while (true)
    {
        int16_t value = adc.readADC(channel);
        std::cout << (channel == 0 ? "A0: " : "A1: ") << value << std::endl;

        // Switch channel for next read
        channel = (channel == 0) ? 1 : 0;

        usleep(100000); // 100 ms delay
    }
    */

    close(file);
    return 0;
}
