/**
 * @file ADS1115.hpp
 * @brief Simplified ADS1115 single-shot interface.
 */

#ifndef ADS1115_H
#define ADS1115_H

#include <cstdint>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <thread>
#include <chrono>
#include <string> // Added for bus path

#define ADS1115_ADDR 0x48
#define ADS1115_CONVERSION_REG 0x00
#define ADS1115_CONFIG_REG 0x01

// Config for single-shot mode, gain = 2/3 (FSR=4.096V), 860 SPS
#define ADS1115_CONFIG_OS_SINGLE 0x8000
#define ADS1115_CONFIG_MUX_A0 0x4000 // AIN0 single-ended
#define ADS1115_CONFIG_MUX_A1 0x5000 // AIN1 single-ended
#define ADS1115_CONFIG_PGA_2_3 0x0000 // FSR +/- 6.144V (actual max is 4.096V with this gain)
#define ADS1115_CONFIG_MODE_SINGLE 0x0100 // Single-shot conversion mode
#define ADS1115_CONFIG_DR_860SPS 0x00E0 // Data Rate 860 Samples per second
#define ADS1115_CONFIG_DEFAULT (ADS1115_CONFIG_PGA_2_3 | ADS1115_CONFIG_MODE_SINGLE | ADS1115_CONFIG_DR_860SPS)

class ADS1115 {
private:
    int file;
    uint8_t i2cAddress;

    // Helper functions to manage I2C communication
    bool setSlaveAddress();
    bool writeReg(uint8_t reg, uint16_t value);
    uint16_t readReg(uint8_t reg);

public:
    /**
     * @brief Constructor. Opens the I2C bus device file.
     * @param busPath The path to the I2C bus (e.g., "/dev/i2c-1").
     * @param address The I2C slave address of the ADS1115 (default 0x48).
     */
    ADS1115(const std::string& busPath, uint8_t address = ADS1115_ADDR);
    
    /**
     * @brief Destructor. Ensures the I2C file descriptor is closed.
     */
    virtual ~ADS1115() { if (file >= 0) close(file); }

    /**
     * @brief Simple check to verify the I2C bus file descriptor was opened successfully.
     * @return true if the driver is ready to communicate, false otherwise.
     */
    bool initialize() const { return file >= 0; } 

    /**
     * @brief Reads a single-shot conversion result from the specified analog channel.
     * This function writes configuration, waits, and reads the 16-bit result.
     * @param channel The analog input channel to read (0 = AIN0, 1 = AIN1).
     * @return The raw signed 16-bit ADC count, or 0 on failure/invalid channel.
     */
    int16_t readADC(uint8_t channel); // 0 = AIN0, 1 = AIN1
};

#endif // ADS1115_H
