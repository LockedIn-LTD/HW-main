#include "MAX30102.hpp"

// Linux I2C and low-level headers
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <numeric>
#include <chrono>
#include <algorithm>

// === I2C Communication Implementation ===

MAX30102::MAX30102(const std::string& i2cDevicePath) : i2cDevPath(i2cDevicePath) {
    // Open the I2C device file
    i2cFileDescriptor = open(i2cDevPath.c_str(), O_RDWR);
    if (i2cFileDescriptor < 0) {
        std::cerr << "Error: Failed to open I2C bus: " << i2cDevPath << std::endl;
    } else {
        // Set the I2C slave address
        if (ioctl(i2cFileDescriptor, I2C_SLAVE, I2C_ADDR) < 0) {
            std::cerr << "Error: Failed to acquire bus access or talk to slave 0x"
                      << std::hex << (int)I2C_ADDR << std::dec << std::endl;
            close(i2cFileDescriptor);
            i2cFileDescriptor = -1;
        }
    }
    // Initialize buffers
    irBuffer.resize(BUFFER_SIZE, 0.0f);
    redBuffer.resize(BUFFER_SIZE, 0.0f);
}

MAX30102::~MAX30102() {
    if (i2cFileDescriptor >= 0) {
        // Reset the sensor on exit
        writeRegister(REG_MODE_CONFIG, 0x40); // MODE_CONFIG: Reset (0x40)
        close(i2cFileDescriptor);
    }
}

int MAX30102::writeRegister(uint8_t regAddr, uint8_t data) {
    if (i2cFileDescriptor < 0) return -1;
    uint8_t buf[2] = {regAddr, data};
    if (write(i2cFileDescriptor, buf, 2) != 2) {
        std::cerr << "I2C Write Error: Reg 0x" << std::hex << (int)regAddr << std::dec << std::endl;
        return -1;
    }
    return 0;
}

int MAX30102::readRegister(uint8_t regAddr, uint8_t& data) {
    if (i2cFileDescriptor < 0) return -1;
    if (write(i2cFileDescriptor, &regAddr, 1) != 1) {
        std::cerr << "I2C Read Error (Write Reg Addr)" << std::endl;
        return -1;
    }
    if (read(i2cFileDescriptor, &data, 1) != 1) {
        std::cerr << "I2C Read Error (Read Data)" << std::endl;
        return -1;
    }
    return 0;
}

int MAX30102::readFifo(uint8_t* buffer, size_t length) {
    if (i2cFileDescriptor < 0) return -1;
    uint8_t regAddr = REG_FIFO_DATA;
    if (write(i2cFileDescriptor, &regAddr, 1) != 1) {
        return -1; // Set pointer
    }
    if (read(i2cFileDescriptor, buffer, length) != (ssize_t)length) {
        return -1; // Read data
    }
    return 0;
}

// === Initialization and Configuration ===

bool MAX30102::initialize() {
    if (i2cFileDescriptor < 0) return false;

    // 1. Check Part ID (should be 0x15)
    uint8_t partID = 0;
    if (readRegister(REG_PART_ID, partID) != 0 || partID != 0x15) {
        std::cerr << "Error: MAX30102 Part ID mismatch. Found 0x" << std::hex << (int)partID << std::dec << " (Expected 0x15)" << std::endl;
        return false;
    }
    std::cout << "MAX30102 found and verified." << std::endl;

    // 2. Reset the chip
    writeRegister(REG_MODE_CONFIG, 0x40); // Reset (0x40)
    usleep(10000); // Wait 10ms for reset to complete

    // 3. FIFO Configuration (REG_FIFO_CONFIG: 0x08)
    // 0x0F: Sample Average = 4, FIFO Rollover = Enable, FIFO almost full = 15
    writeRegister(REG_FIFO_CONFIG, 0x4F);

    // 4. Mode Configuration (REG_MODE_CONFIG: 0x09)
    // 0x03: SpO2 Mode (Red + IR)
    writeRegister(REG_MODE_CONFIG, 0x03);

    // 5. SpO2 Configuration (REG_SPO2_CONFIG: 0x0A)
    // 0x27: ADC Range = 4096nA (0b10), Sample Rate = 100Hz (0b001), Pulse Width = 411us (0b11)
    writeRegister(REG_SPO2_CONFIG, 0x27);

    // 6. Set LED Currents (REG_LED1_PA: 0x0C, REG_LED2_PA: 0x0D)
    // 0x24 = ~7.4mA (Good starting point for many applications)
    writeRegister(REG_LED1_PA, 0x24); // Red
    writeRegister(REG_LED2_PA, 0x24); // IR

    // 7. Enable Interrupts (REG_INT_ENABLE: 0x02)
    // 0x10: FIFO Full (only useful if using the INT pin)
    writeRegister(REG_INT_ENABLE, 0x00); // Clear all interrupts for simplicity

    return true;
}

// === Data Acquisition and Conversion ===

bool MAX30102::readSensor(SensorData& data) {
    if (i2cFileDescriptor < 0) return false;

    // Check how many samples are in the FIFO (optional, but good practice)
    uint8_t writePtr, readPtr;
    readRegister(REG_FIFO_WR_PTR, writePtr);
    readRegister(REG_FIFO_RD_PTR, readPtr);
    int numSamples = (writePtr > readPtr) ? (writePtr - readPtr) : (256 - readPtr + writePtr);

    if (numSamples < 1) {
        return false; // No new data available
    }

    // Read the FIFO (9 bytes per sample: 3 for RED, 3 for IR)
    // We only read 1 sample per call for simplicity in main loop
    uint8_t buffer[6];
    if (readFifo(buffer, 6) != 0) {
        return false;
    }

    // Convert 3 bytes (24-bit) data into 32-bit unsigned integers
    // The first two bits are always 0, so mask is important.
    data.rawRed = (static_cast<uint32_t>(buffer[0] & 0x03) << 16) |
                  (static_cast<uint32_t>(buffer[1]) << 8) |
                  static_cast<uint32_t>(buffer[2]);

    data.rawIR = (static_cast<uint32_t>(buffer[3] & 0x03) << 16) |
                 (static_cast<uint32_t>(buffer[4]) << 8) |
                 static_cast<uint32_t>(buffer[5]);

    // Check for finger presence (a very simple metric)
    // If IR is very low (e.g., < 1000), the finger is likely off.
    data.isFingerPresent = (data.rawIR > 10000);

    // Run the DSP algorithm if a finger is present
    if (data.isFingerPresent) {
        runDSP(data);
    } else {
        // Reset values and state if finger is removed
        data.heartRate = 0.0f;
        data.spO2 = 0.0f;
        // Also clear DSP buffers/state here if needed for clean start
    }

    return true;
}

// === Robust DSP Implementation (Option B) ===

void MAX30102::runDSP(SensorData& data) {
    // Current timestamp in milliseconds (approximation)
    long now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    // 1. DC Subtraction & Low-Pass Filtering (to get AC component)
    // Use a simple exponential moving average (EMA) to estimate the DC component.
    const float ALPHA = 0.95f; // EMA smoothing factor (closer to 1.0 is slower to adapt)
    irDC = irDC * ALPHA + data.rawIR * (1.0f - ALPHA);
    redDC = redDC * ALPHA + data.rawRed * (1.0f - ALPHA);

    // AC signal is the raw value minus the DC estimate.
    float irAC = data.rawIR - irDC;
    float redAC = data.rawRed - redDC;

    // 2. Buffer Management (store AC for HR/SpO2 calculation)
    irBuffer.erase(irBuffer.begin());
    irBuffer.push_back(irAC);
    redBuffer.erase(redBuffer.begin());
    redBuffer.push_back(redAC);

    // 3. Heart Rate Calculation (Peak Detection)
    // We look for a peak in the IR signal (as it's usually stronger/cleaner)
    // A simplified peak detection: look for a rising edge followed by a peak.

    // Calculate a dynamic threshold based on the last few AC peaks (simplified)
    float irMax = *std::max_element(irBuffer.begin(), irBuffer.end());
    float irMin = *std::min_element(irBuffer.begin(), irBuffer.end());
    float peakThreshold = irMin + (irMax - irMin) * 0.5f; // Mid-point threshold

    // Check for a positive zero crossing and a value above the threshold
    if (irBuffer[BUFFER_SIZE - 2] < 0 && irBuffer.back() >= 0 && irBuffer.back() > peakThreshold) {
        // Found a positive-going zero-crossing/peak candidate (a beat)
        long beatTimeDiff = now - lastBeatTimestamp;

        if (lastBeatTimestamp != 0 && beatTimeDiff > 250 && beatTimeDiff < 1800) { // Valid HR range (33-240 BPM)
            // Valid beat found! Update HR.
            float currentPeriodMs = (float)beatTimeDiff;
            // 60000 ms / period_ms = BPM
            currentHR = 60000.0f / currentPeriodMs;

            // Simple running average for smoother HR (20% current, 80% previous)
            data.heartRate = data.heartRate * 0.8f + currentHR * 0.2f;

            // Now that we have a beat, calculate SpO2 at this point
            // SpO2 calculation requires AC and DC components from both Red and IR at the beat time.

            // 4. SpO2 Calculation (Ratio of Ratios, or R value)
            float rNum = (*std::max_element(redBuffer.begin(), redBuffer.end()) - *std::min_element(redBuffer.begin(), redBuffer.end())) / redDC;
            float rDen = (*std::max_element(irBuffer.begin(), irBuffer.end()) - *std::min_element(irBuffer.begin(), irBuffer.end())) / irDC;

            float R = (rDen != 0.0f) ? rNum / rDen : 0.0f;
            
            // Simple running average for smoother R
            lastR = lastR * 0.8f + R * 0.2f;

            // Simplified SpO2 conversion formula (linear approximation)
            // R = 1.0 corresponds roughly to 98% SpO2 (Healthy)
            // R = 1.8 corresponds roughly to 90% SpO2 (Hypoxia)
            // SpO2 = A * R + B (where A and B are empirical constants)
            // Using constants from established open-source projects:
            float spO2Estimate = -45.060f * lastR + 104.981f; // Empirical formula

            // Clamp to reasonable range
            data.spO2 = std::min(std::max(spO2Estimate, 70.0f), 100.0f);
        }
        lastBeatTimestamp = now;
    }
}