#include "I2CDevice.h"
#include <iostream>

I2CDevice::I2CDevice(uint8_t address, const char* i2cBus) {
    i2cFile = open(i2cBus, O_RDWR);
    if (i2cFile < 0) {
        std::cerr << "Failed to open I2C bus" << std::endl;
    }
    if (ioctl(i2cFile, I2C_SLAVE, address) < 0) {
        std::cerr << "Failed to connect to I2C device" << std::endl;
    }
}

I2CDevice::~I2CDevice() {
    if (i2cFile >= 0) close(i2cFile);
}

bool I2CDevice::write(const uint8_t* data, size_t length) {
    return (write(i2cFile, data, length) == (ssize_t)length);
}

bool I2CDevice::read(uint8_t* data, size_t length) {
    return (read(i2cFile, data, length) == (ssize_t)length);
}
