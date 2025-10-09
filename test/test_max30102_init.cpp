#include <TestUtility.hpp> 
#include <MAX30102.hpp>
#include <string>
#include <iostream>

// A mock I2C path that is guaranteed to fail when calling 'open()'
const std::string INVALID_I2C_BUS = "/dev/i2c-INVALID"; 
// Note: MAX30102 constructor only takes one argument (bus path)
// since the address is internally defined in most libraries.

// --- Test Case Definition ---

TEST_CASE(MAX30102_InitializationFailsOnInvalidBus) {
    // 1. Create the object with an invalid bus path.
    MAX30102 max30102(INVALID_I2C_BUS); 

    // 2. We expect initialize() to return false because the I2C bus file descriptor will be invalid.
    EXPECT_FALSE(max30102.initialize());
}
END_TEST_CASE

// --- Main Test Runner ---

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    RUN_ALL_TESTS(test_MAX30102_InitializationFailsOnInvalidBus);
    print_results();
    return 0;
}
