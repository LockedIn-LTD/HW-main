cmake_minimum_required(VERSION 3.10)
project(MAPPLE-WALNUT LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------------------------------------------------
# --- 1. Dependency Management ---
# ------------------------------------------------------------------------

# A. Find essential libraries
find_library(I2C_LIB i2c)
if(NOT I2C_LIB)
    message(FATAL_ERROR "libi2c not found. Install with: sudo apt install libi2c-dev")
endif()

# B. Find cURL (HTTP Client for Firebase REST)
find_package(CURL REQUIRED)
# Install hint for users: sudo apt install libcurl4-openssl-dev

# C. Add nlohmann/json (JSON Handling) using FetchContent
# This downloads the header-only library during configuration.
include(FetchContent)
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3 # Use a specific version tag
)
FetchContent_MakeAvailable(nlohmann_json)


# ------------------------------------------------------------------------
# --- 2. Main Application (MAPPLE-WALNUT) ---
# ------------------------------------------------------------------------

# Sources for the main application
set(APP_SOURCES
    src/ADS1115.cpp
    src/MAX30102.cpp
    src/MPU6050.cpp
    src/main.cpp
)

# Create executable
add_executable(MAPPLE-WALNUT ${APP_SOURCES})

# Include directories
target_include_directories(MAPPLE-WALNUT
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

# Link all dependencies: i2c, pthreads, cURL, and nlohmann::json
target_link_libraries(MAPPLE-WALNUT 
    PRIVATE 
        ${I2C_LIB} 
        pthread 
        CURL::libcurl
        nlohmann_json
)

# Optional: define a macro for bus detection
target_compile_definitions(MAPPLE-WALNUT PRIVATE I2C_BUS_PATH=\"/dev/i2c-7\")

# =========================================================================
# --- Sensor Unit Tests (Individual CTest Registration) ---
# =========================================================================

# 1. Enable testing for this directory
enable_testing()

# Define the common source files required by all tests
set(COMMON_TEST_SOURCES
    src/ADS1115.cpp
    src/MAX30102.cpp
    src/MPU6050.cpp
)

# Define common include directories
set(TEST_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/test
)

# 2. ADS1115 Test
add_executable(test_ads1115 test/test_ads1115_init.cpp ${COMMON_TEST_SOURCES})
target_include_directories(test_ads1115 PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_ads1115 PRIVATE ${I2C_LIB} pthread)
add_test(
    NAME ADS1115_Init_Fails_Gracefully
    COMMAND test_ads1115
)

# 3. MPU6050 Test
add_executable(test_mpu6050 test/test_mpu6050_init.cpp ${COMMON_TEST_SOURCES})
target_include_directories(test_mpu6050 PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_mpu6050 PRIVATE ${I2C_LIB} pthread)
add_test(
    NAME MPU6050_Init_Fails_Gracefully
    COMMAND test_mpu6050
)

# 4. MAX30102 Test
add_executable(test_max30102 test/test_max30102_init.cpp ${COMMON_TEST_SOURCES})
target_include_directories(test_max30102 PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_max30102 PRIVATE ${I2C_LIB} pthread)
add_test(
    NAME MAX30102_Init_Fails_Gracefully
    COMMAND test_max30102
)