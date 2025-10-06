# Set the minimum version of CMake that can be used
# To find the cmake version run
# $ cmake --version
cmake_minimum_required(VERSION 3.10)

# Set the project name
project (MAPPLE-WALNUT)

# Create a sources variable with a link to all cpp files to compile
set(SOURCES
    src/ADS1115.cpp
    src/HeartRate.cpp
    src/JS_HeartRate.cpp
    src/MAX30102.cpp
    src/main.cpp
)

# Add an executable with the above sources
add_executable(MAPPLE-WALNUT ${SOURCES})

# Set the directories that should be included in the build command for this target
# when running g++ these will be included as -I/directory/path/
target_include_directories(MAPPLE-WALNUT
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
)