#include <ADS1115.hpp>
#include <MPU6050.hpp>
#include <MAX30102.hpp> // Assuming this is linked correctly
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <iomanip>
#include <string>
#include <cmath>
#include <thread>
#include <mutex>
// Mutexes for shared resources
std::mutex cout_mutex;
std::mutex i2c_mutex;

using namespace std;

// Define I2C Buses
string I2C_BUS_1 = "/dev/i2c-1";
string I2C_BUS_7 = "/dev/i2c-7";

ADS1115 pressureSensor (I2C_BUS_7, 0x48);
MPU6050 accelerometerSensor (0x68, I2C_BUS_7);
MAX30102 heartbeatSensor (I2C_BUS_7);


void setupSensors() {
    //!accelerometerSensor.calibrate(100) => this one runs its own thread
    accelerometerSensor.calibrate(100);
    if (!pressureSensor.initialize() || !heartbeatSensor.initialize()) {
        std::lock_guard<std::mutex> lock(cout_mutex);
        cout << "Error initializing sensors!" << endl;
        exit(1);
    }
}


void pressureSensorTask() {
    while (true) {
        int16_t adc_value_left, adc_value_right;
        {
            std::lock_guard<std::mutex> lock(i2c_mutex);
            adc_value_left = pressureSensor.readADC(0);
            adc_value_right = pressureSensor.readADC(1);
        }
        {
            std::lock_guard<std::mutex> lock(cout_mutex);
            cout << "Pressure Sensor Readings - Left: " << adc_value_left << " Right: " << adc_value_right << endl;
        }
        this_thread::sleep_for(chrono::milliseconds(3000)); // Adjust the sleep duration as needed
    }
}


void imuSensorTask() {
    while (true) {
        float roll, pitch, yaw;
        float accX, accY, accZ;
        float gyroX, gyroY, gyroZ;
        {   accelerometerSensor.update(); }
        {
            std::lock_guard<std::mutex> lock(i2c_mutex);
            accelerometerSensor.getAngle(0, &roll);
            accelerometerSensor.getAngle(1, &pitch);
            accelerometerSensor.getAngle(2, &yaw);
            accelerometerSensor.getAccel(&accX, &accY, &accZ);
            accelerometerSensor.getGyro(&gyroX, &gyroY, &gyroZ);
        }
        {
            std::lock_guard<std::mutex> lock(cout_mutex);
            cout << "IMU Sensor Readings - Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << endl;
            cout << "Accelerometer - X: " << accX << " Y: " << accY << " Z: " << accZ << endl;
            cout << "Gyroscope - X: " << gyroX << " Y: " << gyroY << " Z: " << gyroZ << endl;
        }
        this_thread::sleep_for(chrono::milliseconds(1000)); // Adjust the sleep duration as needed
    }
}

void heartrateTask(){
    SensorData data = {0};
    while (true) {
        
        bool readSuccess;
        {
            std::lock_guard<std::mutex> lock(i2c_mutex);
            readSuccess = heartbeatSensor.readSensor(data);
        }
        {
            std::lock_guard<std::mutex> lock(cout_mutex);
            if (readSuccess) {
                cout << "Heart Rate Sensor Readings - Heart Rate: " << setw(8) << data.heartRate
                << " BPM, SpO2: " << setw(8) << data.spO2 << "% , Finger Present: "
                << (data.isFingerPresent ? "Yes" : "No") << endl;
            } else {
                cout << "Failed to read from Heart Rate Sensor." << endl;
            }
        }
        this_thread::sleep_for(chrono::milliseconds(10)); // Adjust the sleep duration as needed
    }
}


int main() {
    // create objects for each sensor
    setupSensors();

    //thread pressureSensorRead(pressureSensorTask);
    thread imuSensorRead(imuSensorTask);
    //thread heartrateSensorRead(heartrateTask);

    //pressureSensorRead.join();
    imuSensorRead.join();
    //heartrateSensorRead.join();

    return 0;
}