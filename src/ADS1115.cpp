/**
 * @file ADS1115.cpp
 * @author DriveSense Inc.
 * @brief Contains implementation for the ADS1115 Object.
 */

#include <ADS1115.hpp>

ADS1115::ADS1115(uint8_t address) : file(-1) {};

uint16_t ADS1115::readFromReg(int file, uint8_t reg)
{
    uint8_t buffer[2];

    if (write(file, &reg, 1) != 1)
    {
        printf("Failed to write to reg %p ", reg);
        return 0;
    }

    if (read(file, buffer, 2) != 2)
    {
        printf("Failed to read from reg %p", reg);
        return 0;
    }

    return (buffer[0] << 8) | buffer[1];
}

bool ADS1115::writeToReg(int file, uint8_t reg, uint16_t value)
{
    uint8_t data[3];
    data[0] = reg;          // register we are writing to
    data[1] = value >> 8;   // High byte
    data[2] = value & 0xFF; // Low byte

    if (write(file, data, 3) != 3)
    {
        printf("Failed to write %p to reg %p ", value, reg);
        return false;
    }

    return true;
}

bool ADS1115::initA0()
{
    return writeToReg(this->file, ADS1115_CONFIGURATION_REG, ADS1115_CONTINUOUS_CONFIG__A0);
}

bool ADS1115::initA1()
{
    return writeToReg(this->file, ADS1115_CONFIGURATION_REG, ADS1115_CONTINUOUS_CONFIG_A1);
}

bool ADS1115::setConfig(uint16_t config)
{
    return writeToReg(this->file, ADS1115_CONFIGURATION_REG, config);
}

int16_t ADS1115::readADC()
{
    return static_cast<int16_t>(readFromReg(this->file, ADS1115_CONVERSION_REG));
}