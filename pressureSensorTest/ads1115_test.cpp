#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#define ADS1115_ADDRESS 0x48  // Default I2C address for ADS1115
#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG 0x01

// I2C Bus (Use bus 1 for Jetson Nano)
#define I2C_BUS "/dev/i2c-7"

// Function to write data to ADS1115
void writeToADS1115(int file, uint8_t reg, uint16_t value) {
    uint8_t buffer[3];
    buffer[0] = reg;  // Register address
    buffer[1] = value >> 8;  // High byte
    buffer[2] = value & 0xFF;  // Low byte

    if (write(file, buffer, 3) != 3) {
        std::cerr << "Failed to write to ADS1115" << std::endl;
    }
}

// Function to read data from ADS1115
uint16_t readFromADS1115(int file, uint8_t reg) {
    uint8_t buffer[2];
    if (write(file, &reg, 1) != 1) {
        std::cerr << "Failed to set register" << std::endl;
        return 0;
    }
    if (read(file, buffer, 2) != 2) {
        std::cerr << "Failed to read from ADS1115" << std::endl;
        return 0;
    }
    return (buffer[0] << 8) | buffer[1];
}

int main() {
    // Open the I2C bus
    while (true) {
    int file = open(I2C_BUS, O_RDWR);
    if (file < 0) {
        std::cerr << "Failed to open I2C bus." << std::endl;
        return -1;
    }

    // Specify the I2C address of the ADS1115
    if (ioctl(file, I2C_SLAVE, ADS1115_ADDRESS) < 0) {
        std::cerr << "Failed to connect to the I2C device." << std::endl;
        return -1;
    }

    // Configure the ADS1115 (e.g., single-ended mode, 16-bit, etc.)
    uint16_t config = 0x8583;  // Example config for 16-bit, single-ended, 4.096V, continuous mode
    writeToADS1115(file, ADS1115_REG_CONFIG, config);
    usleep(100000);  // Wait for conversion to complete

    // Read conversion result (16-bit)
    uint16_t rawData = readFromADS1115(file, ADS1115_REG_CONVERSION);

    // Print the result (ADC value)
    std::cout << "ADC Value: " << rawData << std::endl;

    // Close the I2C device
    close(file);
}
    return 0;
}

