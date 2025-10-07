/**
 * @file ADS1115.hpp
 * @author DriveSense Inc.
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

#define ADS1115_ADDR 0x48
#define ADS1115_CONVERSION_REG 0x00
#define ADS1115_CONFIG_REG     0x01

// Config for single-shot mode, gain = 2/3, 860 SPS
#define ADS1115_CONFIG_OS_SINGLE   0x8000
#define ADS1115_CONFIG_MUX_A0      0x4000
#define ADS1115_CONFIG_MUX_A1      0x5000
#define ADS1115_CONFIG_PGA_2_3     0x0000
#define ADS1115_CONFIG_MODE_SINGLE 0x0100
#define ADS1115_CONFIG_DR_860SPS   0x00E0
#define ADS1115_CONFIG_DEFAULT     (ADS1115_CONFIG_PGA_2_3 | ADS1115_CONFIG_MODE_SINGLE | ADS1115_CONFIG_DR_860SPS)

class ADS1115 {
private:
    int file;
    uint8_t i2cAddress;

    bool setSlaveAddress();
    bool writeReg(uint8_t reg, uint16_t value);
    uint16_t readReg(uint8_t reg);

public:
    ADS1115(int fd = -1, uint8_t address = ADS1115_ADDR);

    void setFile(int fd);

    int16_t readADC(uint8_t channel); // 0 = A0, 1 = A1
};

#endif // ADS1115_H
