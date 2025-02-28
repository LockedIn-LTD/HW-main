#include "ADS1115.hpp"
#include <iostream>
#include <cstdint>


ADS1115::ADS1115(uint8_t address) : i2cDevice(address) {};


bool ADS1115::init() {
    uint16_t config = 0x0483;  // AIN0, ±4.096V, continuous mode, 128 SPS
    return writeRegister(ADS1115_CONFIGURATION_REG, config);
}

int16_t ADS1115::readADC() {
    return readRegister(ADS1115_CONVERSION_REG);
}


void ADS1115::setConfig(uint16_t config) {
    writeRegister(ADS1115_CONFIGURATION_REG, config);
}


bool ADS1115::writeRegister(uint8_t reg, uint16_t value) {
    uint8_t data[3] = {reg, static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
    return i2cDevice.write(data, 3);
}


uint16_t ADS1115::readRegister(uint8_t reg) {
    i2cDevice.write(&reg, 1);
    uint8_t data[2] = {0};
    i2cDevice.read(data, 2);
    return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}