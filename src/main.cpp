#include <ADS1115.hpp>
#include <iostream>
#include <cstdio>
#include <sys/ioctl.h>

// Bus 7 on the jetson nano
#define I2C_BUS "/dev/i2c-7"

int main()
{

    int file = open(I2C_BUS, O_RDWR);

    if (file < 0)
    {
        printf("Failed to create file to start I2C.");
        return 0;
    }


    ADS1115 pressureSensor;

    pressureSensor.setFile(file);

    // Specify the I2C address of the ADS1115
    if (ioctl(file, I2C_SLAVE, ADS1115_ADDRS) < 0)
    {
        std::cerr << "Failed to connect to the I2C device." << std::endl;
        return -1;
    }


    int i = 1;

    while (i > 0)
    {   
        if (i % 2 == 0) {
            i = 1;
            if (pressureSensor.initA0()){
                printf("A0: ");
            }
        } else {
            i = 2;
            if (pressureSensor.initA1()){
                printf("A1: ");
            }
        }
        
        uint16_t rawData = pressureSensor.readADC();
        std::cout << rawData << std::endl;

        
        usleep(1000);
    }

    close(file);
    return 0;
}