#include <TestUtility.hpp> 
#include <ADS1115.hpp>
#include <string>
#include <iostream>

// A mock I2C path that is guaranteed to fail when calling 'open()'
const std::string INVALID_I2C_BUS = "/dev/i2c-INVALID"; 
const uint8_t TEST_ADDR = 0x01; // A dummy address

// --- Test Case Definition ---

TEST_CASE(ADS1115_InitializationFailsOnInvalidBus) {
    // 1. The constructor will attempt to call open(INVALID_I2C_BUS).
    ADS1115 adc(INVALID_I2C_BUS, TEST_ADDR); 

    // 2. We expect it to report failure since 'open()' would have failed.
    EXPECT_FALSE(adc.initialize());
}
END_TEST_CASE

// --- Main Test Runner ---

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    RUN_ALL_TESTS(test_ADS1115_InitializationFailsOnInvalidBus);
    print_results();
    return 0;
}
