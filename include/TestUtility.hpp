#ifndef TEST_UTILITY_HPP
#define TEST_UTILITY_HPP

#include <iostream>
#include <string>
#include <cstdlib>

// Global counters for test statistics
static int g_test_count = 0;
static int g_test_failures = 0;
static int g_assertion_count = 0;

/**
 * @brief Macro to define and start a new test case function.
 */
#define TEST_CASE(name) \
    void test_##name() { \
        const char* test_name_str = #name; /* Define local variable for macro use */ \
        g_test_count++; \
        std::cerr << "--- Running Test Case: " << test_name_str << " ---" << std::endl; \
        bool failed_in_case = false;

/**
 * @brief Macro to end a test case function.
 */
#define END_TEST_CASE \
        if (!failed_in_case) { \
            /* Use the local string variable defined in TEST_CASE */ \
            std::cerr << "--- Test Case: " << test_name_str << " PASSED ---" << std::endl; \
        } \
    }

/**
 * @brief Assert that a condition is true.
 */
#define EXPECT_TRUE(condition) \
    g_assertion_count++; \
    if (!(condition)) { \
        g_test_failures++; \
        failed_in_case = true; \
        std::cerr << "  [FAILURE] Assertion failed at " << __FILE__ << ":" << __LINE__ \
                  << ": " << #condition << std::endl; \
    } else { \
        std::cerr << "  [SUCCESS] Assertion passed: " << #condition << std::endl; \
    }

/**
 * @brief Assert that a condition is false.
 */
#define EXPECT_FALSE(condition) EXPECT_TRUE(!(condition))

/**
 * @brief Utility function to run a single test case function.
 */
#define RUN_ALL_TESTS(func) func();

/**
 * @brief Prints the final test summary and exits with status 0 on success, 1 on failure.
 */
static void print_results() {
    std::cerr << "\n========================================" << std::endl;
    std::cerr << "TEST SUMMARY" << std::endl;
    std::cerr << "Total Test Cases Run: " << g_test_count << std::endl;
    std::cerr << "Total Assertions: " << g_assertion_count << std::endl;
    std::cerr << "Total Failures: " << g_test_failures << std::endl;
    std::cerr << "========================================" << std::endl;

    if (g_test_failures > 0) {
        std::cerr << "TESTS FAILED!" << std::endl;
        exit(1);
    } else {
        std::cerr << "All tests passed successfully." << std::endl;
        exit(0);
    }
}

#endif // TEST_UTILITY_HPP
