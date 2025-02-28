#ifndef I2CDEVICE_H
#define I2CDEVICE_H

#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

class I2CDevice {
public:
    I2CDevice(uint8_t address, const char* i2cBus = "/dev/i2c-1");
    ~I2CDevice();
    bool write(const uint8_t* data, size_t length);
    bool read(uint8_t* data, size_t length);

private:
    int i2cFile;
};

#endif // I2CDEVICE_H
