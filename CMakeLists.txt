cmake_minimum_required(VERSION 3.10)
project(MAPPLE-WALNUT LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Sources
set(SOURCES
    src/ADS1115.cpp
    src/MAX30102.cpp
    src/MPU6050.cpp
    src/main.cpp
)

# Create executable
add_executable(MAPPLE-WALNUT ${SOURCES})

# Include directories
target_include_directories(MAPPLE-WALNUT
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

# Link libraries
find_library(I2C_LIB i2c)
if(NOT I2C_LIB)
    message(FATAL_ERROR "libi2c not found. Install with: sudo apt install libi2c-dev")
endif()

target_link_libraries(MAPPLE-WALNUT PRIVATE ${I2C_LIB} pthread)

# Optional: define a macro for bus detection
target_compile_definitions(MAPPLE-WALNUT PRIVATE I2C_BUS_PATH=\"/dev/i2c-7\")
