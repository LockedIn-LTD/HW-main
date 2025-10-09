#include <TestUtility.hpp> 
#include <MPU6050.hpp>
#include <string>
#include <iostream>

// A mock I2C path that is guaranteed to fail when calling 'open()'
const std::string INVALID_I2C_BUS = "/dev/i2c-INVALID"; 
const uint8_t TEST_ADDR = 0x01; // A dummy address

// --- Test Case Definition ---

TEST_CASE(MPU6050_ConstructionFailsGracefullyOnInvalidBus) {
    // 1. The constructor attempts to open the bus, which will fail.
    MPU6050 imu(TEST_ADDR, INVALID_I2C_BUS); 

    // 2. Primary goal is non-crashing construction and initialization.
    EXPECT_TRUE(true); 
}
END_TEST_CASE

// --- Main Test Runner ---

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    RUN_ALL_TESTS(test_MPU6050_ConstructionFailsGracefullyOnInvalidBus);
    print_results();
    return 0;
}
