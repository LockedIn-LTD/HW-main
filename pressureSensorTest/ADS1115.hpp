/**
 * @file ADS1115.hpp
 * @author DriveSense Inc. 
 * @brief Contains all formal definitions for the ADS1115 Object.
 */

 #ifndef ADS1115_H
 #define ADS1115_H
 
 #include <cstdint>
 #include "I2CDevice.hpp"

 
 #define ADS1115_ADDRS 0x48  // ADDR pin is connected to GND.
 
 #define ADS1115_CONVERSION_REG 0x00  // Contains the conversion of the last stored value.
 #define ADS1115_CONFIGURATION_REG 0x01  // Register to set configuration.
 
 class I2CDevice; // Forward declaration (or include the actual header file)
 
 class ADS1115 {
 public:
     explicit ADS1115(uint8_t address = ADS1115_ADDRS);
     bool init(); // Initialize the ADS sensor
     int16_t readADC();
     void setConfig(uint16_t config);
 
 private:
     I2CDevice i2cDevice;  // Assuming this is an existing class
     bool writeRegister(uint8_t reg, uint16_t value);
     uint16_t readRegister(uint8_t reg);
 };
 
 #endif  // ADS1115_H
 