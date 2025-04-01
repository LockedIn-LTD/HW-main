#include "ADS1115/ADS1115.hpp"
#include "MAX30102/JS_HeartRate.cpp"
#include <iostream>
#include <cstdio>
#include <sys/ioctl.h>

// Bus 7 on the jetson nano 
#define I2C_BUS "/dev/i2c-7"

int main() {

    int file = open(I2C_BUS, O_RDWR);
    
    if (file < 0) {
        printf("Failed to create file to start I2C.");
        return 0;
    }

    // HRSInterface hrs;
    // // MAX30102 I2C channel set in MAX30102.hpp
    // hrs.start();

    ADS1115 pressureSensor;

    pressureSensor.setFile(file);
    
    if (pressureSensor.init()) {
        printf("Succesfully initialized the pressure sensor!");
    } else {
        printf("Failed to initialize the pressure sensor");
        return 0;
    }

    // Specify the I2C address of the ADS1115
    if (ioctl(file, I2C_SLAVE, ADS1115_ADDRS) < 0) {
        std::cerr << "Failed to connect to the I2C device." << std::endl;
        return -1;
    }

    int i = 10; 

    while ( i > 0) {
        uint16_t rawData = pressureSensor.readADC();
        std::cout << "ADC Value: " << rawData << std::endl;

        // // Read IR Heart Rate
        // std::cout << "IR Heart Rate- Latest:" << hrs.getLatestHeartRate() << ", SAFE:" << hrs.getSafeHeartRate()<< std::endl;
        
        i--;
    }

    close(file);
    return 0;

}