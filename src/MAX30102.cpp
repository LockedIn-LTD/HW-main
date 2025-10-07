#include <MAX30102.hpp>
#include <cmath>
#include <sys/ioctl.h>

#define MAX30102_ADDR 0x57

MAX30102::MAX30102(const std::string& i2cBus) : bus(i2cBus), file(-1) {}

MAX30102::~MAX30102() {
    if(file >= 0) close(file);
}

bool MAX30102::begin() {
    file = open(bus.c_str(), O_RDWR);
    if(file < 0) return false;

    if(ioctl(file, I2C_SLAVE, MAX30102_ADDR) < 0) return false;

    reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    configureSensor();
    return true;
}

void MAX30102::reset() {
    writeRegister(0x09, 0x00); // LED1_PA
    writeRegister(0x0A, 0x00); // LED2_PA
    writeRegister(0x0B, 0x00); // LED3_PA

    writeRegister(0x09, 0x24); // Red LED ~36mA
    writeRegister(0x0A, 0x24); // IR LED ~36mA

    writeRegister(0x06, 0x40); // Reset command
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void MAX30102::configureSensor() {
    // Clear FIFO
    writeRegister(0x04, 0x00); // FIFO_WR_PTR
    writeRegister(0x05, 0x00); // OVF_COUNTER
    writeRegister(0x06, 0x00); // FIFO_RD_PTR

    // Mode: Heart Rate
    writeRegister(0x09, 0x24); // LED1_PA Red
    writeRegister(0x0A, 0x24); // LED2_PA IR
    writeRegister(0x06, 0x02); // Mode HR only

    // SPO2 config: sample rate 100Hz, LED pulse width 411us
    writeRegister(0x0A, 0x03); // ADC resolution
}

void MAX30102::startHeartRate() {
    // Nothing extra needed for now; mode already set
}

void MAX30102::getFIFO(uint32_t &red, uint32_t &ir) {
    // Read FIFO (example: one sample)
    uint8_t buf[6];
    if(i2c_smbus_read_i2c_block_data(file, 0x07, 6, buf) < 0) {
        red = ir = 0;
        return;
    }

    ir  = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    red = ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5];

    lastRed = red;
    lastIR  = ir;

    // Simple BPM placeholder (replace with algorithm later)
    bpm = 60 + (red % 40); 
}

bool MAX30102::writeRegister(uint8_t reg, uint8_t value) {
    return i2c_smbus_write_byte_data(file, reg, value) >= 0;
}

uint8_t MAX30102::readRegister(uint8_t reg) {
    int val = i2c_smbus_read_byte_data(file, reg);
    return (val >= 0) ? val : 0;
}

float MAX30102::readTemperature() {
    return 30.0f + (rand() % 1000) / 100.0f; // dummy for now
}