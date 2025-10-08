#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <cstdint>

// Forward declaration for Linux I2C functions
// We put the heavy lifting headers in the .cpp file

/**
 * @brief Structure to hold the sensor and calculated data.
 */
struct SensorData {
    uint32_t rawIR;
    uint32_t rawRed;
    float heartRate; // Beats per Minute (BPM)
    float spO2;      // Blood Oxygen Saturation (%)
    bool isFingerPresent; // Simple status check
};

class MAX30102 {
public:
    /**
     * @brief Constructor.
     * @param i2cDevicePath The path to the I2C device (e.g., "/dev/i2c-7").
     */
    MAX30102(const std::string& i2cDevicePath);

    /**
     * @brief Destructor. Closes the I2C file descriptor.
     */
    ~MAX30102();

    /**
     * @brief Initializes the sensor by configuring the operating mode,
     * sampling rate, LED pulse width, and current.
     * @return true on success, false on failure.
     */
    bool initialize();

    /**
     * @brief Reads the FIFO, updates raw values, and runs the DSP algorithm.
     * @param data Reference to the SensorData structure to fill.
     * @return true if data was read and processed, false otherwise.
     */
    bool readSensor(SensorData& data);

private:
    std::string i2cDevPath;
    int i2cFileDescriptor = -1;
    const uint8_t I2C_ADDR = 0x57;

    // -- Sensor Configuration Constants --
    const uint8_t REG_INT_STATUS = 0x00;
    const uint8_t REG_INT_ENABLE = 0x02;
    const uint8_t REG_FIFO_WR_PTR = 0x04;
    const uint8_t REG_OVF_COUNTER = 0x05;
    const uint8_t REG_FIFO_RD_PTR = 0x06;
    const uint8_t REG_FIFO_DATA = 0x07;
    const uint8_t REG_FIFO_CONFIG = 0x08;
    const uint8_t REG_MODE_CONFIG = 0x09;
    const uint8_t REG_SPO2_CONFIG = 0x0A;
    const uint8_t REG_LED1_PA = 0x0C; // Red LED Current
    const uint8_t REG_LED2_PA = 0x0D; // IR LED Current
    const uint8_t REG_PART_ID = 0xFF;

    // -- DSP Variables (for Option B robustness) --
    // Circular buffers for raw data
    static const size_t BUFFER_SIZE = 100;
    std::vector<float> irBuffer;
    std::vector<float> redBuffer;

    // DC filtering variables
    float irDC = 0.0f;
    float redDC = 0.0f;

    // Heart Rate calculation
    std::vector<long> pulseTimestamps;
    long lastBeatTimestamp = 0;
    float currentHR = 0.0f;

    // SpO2 calculation
    float lastR = 0.0f; // Last calculated R-value

    // -- Private Methods --

    /**
     * @brief Low-level I2C read from a single register.
     */
    int readRegister(uint8_t regAddr, uint8_t& data);

    /**
     * @brief Low-level I2C write to a single register.
     */
    int writeRegister(uint8_t regAddr, uint8_t data);

    /**
     * @brief Reads multiple bytes from the FIFO data register.
     */
    int readFifo(uint8_t* buffer, size_t length);

    /**
     * @brief Runs the robust DSP algorithm for HR and SpO2.
     */
    void runDSP(SensorData& data);
};