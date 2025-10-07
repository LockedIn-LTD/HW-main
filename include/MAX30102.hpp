#pragma once
#include <string>
#include <cstdint>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>

extern "C"
{
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

class MAX30102 {
public:
    explicit MAX30102(const std::string& i2cBus);
    ~MAX30102();

    bool begin();
    void startHeartRate();
    void getFIFO(uint32_t &red, uint32_t &ir);
    uint32_t getRed() const { return lastRed; }
    uint32_t getIR() const { return lastIR; }
    uint32_t getLatestBPM() const { return bpm; }
    float readTemperature();

private:
    std::string bus;
    int file;
    uint32_t lastRed = 0;
    uint32_t lastIR = 0;
    uint32_t bpm = 0;

    bool writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void reset();
    void configureSensor();
};