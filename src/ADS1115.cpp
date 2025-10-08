#include <ADS1115.hpp>

// MODIFIED: Constructor now takes busPath and manages its own file descriptor
ADS1115::ADS1115(const std::string& busPath, uint8_t address)
    : file(-1), i2cAddress(address)
{
    // Open I2C bus
    file = open(busPath.c_str(), O_RDWR);
    if (file < 0) {
        printf("Failed to open I2C bus: %s\n", busPath.c_str());
        return;
    }
}

bool ADS1115::setSlaveAddress()
{
    if (file < 0) return false;
    // CRITICAL: Re-set slave address for every read due to shared bus (Bus 2)
    if (ioctl(file, I2C_SLAVE, i2cAddress) < 0) {
        perror("ioctl I2C_SLAVE failed for ADS1115");
        return false;
    }
    return true;
}

bool ADS1115::writeReg(uint8_t reg, uint16_t value)
{
    if (file < 0) return false;
    uint8_t buffer[3] = { reg, uint8_t(value >> 8), uint8_t(value & 0xFF) };
    if (write(file, buffer, 3) != 3) {
        printf("Failed to write %04X to reg %02X (ADS1115)\n", value, reg);
        return false;
    }
    return true;
}

uint16_t ADS1115::readReg(uint8_t reg)
{
    if (file < 0) return 0;
    
    // 1. Write register address (select the register to read)
    if (write(file, &reg, 1) != 1) {
        printf("Failed to write reg %02X for read (ADS1115)\n", reg);
        return 0;
    }
    
    // 2. Read 2 bytes (16-bit value)
    uint8_t buf[2];
    if (read(file, buf, 2) != 2) {
        printf("Failed to read reg %02X (ADS1115)\n", reg);
        return 0;
    }
    return (buf[0] << 8) | buf[1];
}

int16_t ADS1115::readADC(uint8_t channel)
{
    if (file < 0) return 0;
    
    // Step 1: Acquire bus access, since the bus is shared
    if (!setSlaveAddress()) {
        return 0;
    }
    
    // Step 2: Configure for single-shot conversion
    uint16_t mux;
    if (channel == 0) {
        mux = ADS1115_CONFIG_MUX_A0;
    } else if (channel == 1) {
        mux = ADS1115_CONFIG_MUX_A1;
    } else {
        printf("Invalid ADC channel: %d\n", channel);
        return 0;
    }

    uint16_t config = ADS1115_CONFIG_OS_SINGLE | mux | ADS1115_CONFIG_DEFAULT;

    // Start conversion
    if (!writeReg(ADS1115_CONFIG_REG, config)) {
        return 0;
    }

    // Step 3: Wait for conversion to complete (~1.2ms at 860 SPS)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    
    // Step 4: Read conversion result
    return static_cast<int16_t>(readReg(ADS1115_CONVERSION_REG));
}
