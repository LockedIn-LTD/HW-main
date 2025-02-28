#include <iostream>
#include <unistd.h>
#include "ADS1115.hpp"
#include <pigpio.h>

int main() {
    // Initialize pigpio
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialization failed." << std::endl;
        return -1;
    }

    // Open the I2C bus and get the handle (assuming I2C bus 1)
    int i2cHandle = i2cOpen(1, ADS1115_ADDRS, 0);
    if (i2cHandle < 0) {
        std::cerr << "Failed to open I2C device." << std::endl;
        gpioTerminate();
        return -1;
    }

    // Configure the ADS1115 (set gain, sample rate, etc.)
    char config[3] = {0};
    config[0] = 0xC3;  // Example: Configuration byte for ADS1115
    config[1] = 0x83;
    config[2] = 0x40;
    i2cWriteDevice(i2cHandle, config, 3);  // Send configuration to ADS1115

    // Read conversion register (16-bit value)
    char buffer[2];
    i2cReadDevice(i2cHandle, buffer, 2);

    // Convert the data
    int rawData = (buffer[0] << 8) | buffer[1];
    std::cout << "ADC Value: " << rawData << std::endl;

    // Close the I2C device
    i2cClose(i2cHandle);

    // Cleanup pigpio
    gpioTerminate();

    return 0;
}
