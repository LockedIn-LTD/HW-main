#include <ADS1115.hpp>
#include <MPU6050.hpp>
#include <MAX30102.hpp> // Assuming this is linked correctly
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <iomanip>
#include <string>
#include <cmath>

// Define I2C Buses
// I2C_BUS_1: /dev/i2c-1 (Used for MAX30102 (2) only)
const std::string I2C_BUS_1 = "/dev/i2c-1"; 
// I2C_BUS_2: /dev/i2c-7 (Used for MPU6050, ADS1115, and MAX30102 (1))
const std::string I2C_BUS_2 = "/dev/i2c-7"; 

// Slower sensors will update every N cycles (e.g., 10 cycles = 100ms update if loop is 100Hz)
const int SLOW_SENSOR_UPDATE_RATE = 10; 

// Simple struct to hold MPU and ADS data for unified printing
struct MPUReadout { float roll, pitch, yaw; float accX, accY, accZ; float gyroX, gyroY, gyroZ; };
struct ADSReadout { int16_t a0 = 0, a1 = 0; }; // Initialize to zero

int main() {
    // --- 1. Initialization (All drivers manage their own bus access) ---
    
    // MAX30102 Sensors (Explicit addresses based on i2cdetect)
    // MAX30102(2) is on I2C_BUS_1 (/dev/i2c-1) detected 0x57
    MAX30102 max30102_2(I2C_BUS_1); 
    // MAX30102(1) is on I2C_BUS_2 (/dev/i2c-7) detected 0x57
    MAX30102 max30102_1(I2C_BUS_2); 

    // MPU6050 IMU (on I2C_BUS_2: /dev/i2c-7)
    // Bus 7 detected 0x68
    MPU6050 mpu6050(0x68, I2C_BUS_2);
    
    // ADS1115 ADC (on I2C_BUS_2: /dev/i2c-7) -> CORRECTED BUS ASSIGNMENT
    ADS1115 ads1115(I2C_BUS_2, 0x48);

    // Check all initializations
    if (!max30102_1.initialize()) { std::cerr << "Fatal Error: MAX30102 (1) failed on " << I2C_BUS_2 << std::endl; return 1; }
    if (!max30102_2.initialize()) { std::cerr << "Fatal Error: MAX30102 (2) failed on " << I2C_BUS_1 << std::endl; return 1; }
    if (!ads1115.initialize()) { std::cerr << "Fatal Error: ADS1115 failed on " << I2C_BUS_2 << std::endl; return 1; } // Error message updated
    
    std::cout << "All drivers initialized successfully." << std::endl;

    // Calibrate MPU6050 once (must be stationary! This also starts its background thread)
    std::cout << "Calibrating MPU6050... Keep sensor still for 1 second." << std::endl;
    mpu6050.calibrate(100);
    std::cout << "Calibration complete. Entering main loop." << std::endl;


    // --- 2. Data Structures ---
    SensorData data_max1 = {0}; // MAX30102 (on I2C_BUS_2)
    SensorData data_max2 = {0}; // MAX30102 (on I2C_BUS_1)
    MPUReadout data_mpu = {0};  // MPU6050
    ADSReadout data_ads = {0};  // ADS1115

    // --- 3. Setup Console Output ---
    int read_counter = 0;
    uint8_t ads_channel = 0; // Toggle between A0 (0) and A1 (1)

    std::cout << "\n---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "Sensor    | HR (BPM) | SpO2 (%) | Roll (°) | Pitch(°)| ADC (A0) | ADC (A1) | IR / AccX | RED / GyroX" << std::endl;
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;

    // The entire loop runs at the fastest rate (~100Hz)
    while (true) {
        
        // A. Read MAX30102 Sensors (High Speed)
        max30102_1.readSensor(data_max1);
        max30102_2.readSensor(data_max2);

        // B. Read Slower Sensors (Decimated Speed - Runs at 10Hz)
        if (++read_counter >= SLOW_SENSOR_UPDATE_RATE) {
            
            // 1. ADS1115 reading (alternating channels)
            int16_t adc_value = ads1115.readADC(ads_channel);
            if (ads_channel == 0) {
                data_ads.a0 = adc_value;
                ads_channel = 1; // Prepare for next read (A1)
            } else {
                data_ads.a1 = adc_value;
                ads_channel = 0; // Prepare for next read (A0)
            }
            
            // 2. MPU6050 (Retrieve the latest filtered angle from the background thread)
            mpu6050.getAngle(0, &data_mpu.roll);
            mpu6050.getAngle(1, &data_mpu.pitch);
            mpu6050.getAngle(2, &data_mpu.yaw); 
            
            // Get current raw data (needs to re-read MPU, but is throttled here)
            mpu6050.getAccel(&data_mpu.accX, &data_mpu.accY, &data_mpu.accZ);
            mpu6050.getGyro(&data_mpu.gyroX, &data_mpu.gyroY, &data_mpu.gyroZ);

            read_counter = 0;
        }

        // C. Print Consolidated Output (3 lines, overwritten in place)
        
        // Line 1: MAX30102 (1) + MPU Angles + ADC Readings
        std::cout << "MAX30102(1)| " << std::fixed << std::setprecision(2)
                  << std::setw(8) << (data_max1.isFingerPresent ? data_max1.heartRate : 0.0f)
                  << " | " << std::setw(6) << (data_max1.isFingerPresent ? data_max1.spO2 : 0.0f)
                  << " | " << std::setw(8) << data_mpu.roll
                  << " | " << std::setw(7) << data_mpu.pitch
                  << " | " << std::setw(8) << data_ads.a0
                  << " | " << std::setw(8) << data_ads.a1
                  << " | " << std::setw(9) << data_max1.rawIR
                  << " | " << std::setw(9) << data_max1.rawRed
                  << "\n"; 

        // Line 2: MAX30102 (2) + MPU Raw Accel/Gyro data (for diagnostics)
        std::cout << "MAX30102(2)| " << std::fixed << std::setprecision(2)
                  << std::setw(8) << (data_max2.isFingerPresent ? data_max2.heartRate : 0.0f)
                  << " | " << std::setw(6) << (data_max2.isFingerPresent ? data_max2.spO2 : 0.0f)
                  << " | " << std::setw(8) << data_mpu.gyroX 
                  << " | " << std::setw(7) << data_mpu.accY 
                  << " | " << std::setw(8) << "" 
                  << " | " << std::setw(8) << "" 
                  << " | " << std::setw(9) << data_max2.rawIR
                  << " | " << std::setw(9) << data_max2.rawRed
                  << "\n"; 

        // Line 3: Status Line 
        std::cout << "Status: M1: " << (data_max1.isFingerPresent ? "MEASURING" : "FINGER OFF")
                  << " | M2: " << (data_max2.isFingerPresent ? "MEASURING" : "FINGER OFF")
                  << " | MPU Roll: " << std::fixed << std::setprecision(1) << data_mpu.roll
                  << " | ADC A0: " << data_ads.a0
                  << std::string(10, ' ') << "\n"; // Padding to clear old content
        
        // Move the cursor back up three lines to overwrite in place
        std::cout << "\033[A\033[A\033[A";
        std::cout.flush();

        // Control the loop speed to match the fastest sensor rate (~100Hz)
        usleep(8000); // 8ms delay
    }

    return 0;
}
