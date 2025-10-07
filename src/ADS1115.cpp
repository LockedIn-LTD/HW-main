#include <ADS1115.hpp>

ADS1115::ADS1115(int fd, uint8_t address)
    : file(fd), i2cAddress(address)
{
    if (file >= 0 && !setSlaveAddress()) {
        printf("Failed to set I2C address %02X\n", i2cAddress);
    }
}

void ADS1115::setFile(int fd)
{
    file = fd;
    if (!setSlaveAddress()) {
        printf("Failed to set I2C address %02X\n", i2cAddress);
    }
}

bool ADS1115::setSlaveAddress()
{
    if (ioctl(file, I2C_SLAVE, i2cAddress) < 0) {
        perror("ioctl I2C_SLAVE failed");
        return false;
    }
    return true;
}

bool ADS1115::writeReg(uint8_t reg, uint16_t value)
{
    uint8_t buffer[3] = { reg, uint8_t(value >> 8), uint8_t(value & 0xFF) };
    if (write(file, buffer, 3) != 3) {
        printf("Failed to write %04X to reg %02X\n", value, reg);
        return false;
    }
    return true;
}

uint16_t ADS1115::readReg(uint8_t reg)
{
    uint8_t buf[2];
    if (write(file, &reg, 1) != 1) {
        printf("Failed to write reg %02X\n", reg);
        return 0;
    }
    if (read(file, buf, 2) != 2) {
        printf("Failed to read reg %02X\n", reg);
        return 0;
    }
    return (buf[0] << 8) | buf[1];
}

int16_t ADS1115::readADC(uint8_t channel)
{
    uint16_t mux = (channel == 0) ? ADS1115_CONFIG_MUX_A0 : ADS1115_CONFIG_MUX_A1;
    uint16_t config = ADS1115_CONFIG_OS_SINGLE | mux | ADS1115_CONFIG_DEFAULT;

    if (!writeReg(ADS1115_CONFIG_REG, config)) {
        printf("Failed to start conversion on channel %d\n", channel);
        return 0;
    }

    // Wait for conversion (approx 1.2ms for 860 SPS)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    return static_cast<int16_t>(readReg(ADS1115_CONVERSION_REG));
}
