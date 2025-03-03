/**
* @file ADS1115.hpp
* @author DriveSense Inc. 
* @brief Contains all formal definitions for the ADS1115 Object.
*/

#ifndef ADS1115_H
#define ADS1115_H

#include <cstdint>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>

//ADS1115 Defintions
#define ADS1115_ADDRS 0x48  // ADDR pin is connected to GND.
#define ADS1115_CONVERSION_REG 0x00  // Contains the conversion of the last stored value.
#define ADS1115_CONFIGURATION_REG 0x01  // Register to set configuration.

//ADS1115 Configurations
#define ADS1115_DEFAULT_CONFIG 0x8383 
#define ADS1115_GAIN_1_CONFIG 0x8583 
#define ADS1115_GAIN_2_CONFIG 0x8783 
#define ADS1115_GAIN_3_CONFIG 0x8983 
#define ADS1115_GAIN_4_CONFIG 0x8B83 
#define ADS1115_GAIN_5_CONFIG 0x8D83 



class ADS1115 {

    private:
        int file;
        bool writeToReg(int file, uint8_t reg, uint16_t value);
        uint16_t readFromReg(int file, uint8_t reg);


    public:
        ADS1115(uint8_t address = ADS1115_ADDRS);
        bool init(); // Initialize the ADS sensor
        int16_t readADC();
        bool setConfig(uint16_t config);
        int getFile() { return file;}
        void setFile(int f) { file = f;}

};

#endif  // ADS1115_H
 