#include <ADS1115.hpp>
#include <MPU6050.hpp>
#include <MAX30102.hpp>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <iomanip>
#include <string>
#include <cmath>
#include <thread>
#include <mutex>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <zmq.hpp>

using json = nlohmann::json;

struct AggregateSensorData {
    // MAX30102 (Heartrate 1)
    float hr1_heartRate = 0.0f;
    float hr1_spO2 = 0.0f;
    bool hr1_isFingerPresent = false;
    // MAX30102 (Heartrate 2)
    float hr2_heartRate = 0.0f;
    float hr2_spO2 = 0.0f;
    bool hr2_isFingerPresent = false;
    // MPU6050
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float accX = 0.0f;
    float accY = 0.0f;
    float accZ = 0.0f;
    float gyroX = 0.0f;
    float gyroY = 0.0f;
    float gyroZ = 0.0f;
    // ADS1115 (Pressure)
    int16_t pressure_left = 0;
    int16_t pressure_right = 0;
};

// Mutexes for shared resources
std::mutex g_data_mutex;
std::mutex cout_mutex;
std::mutex i2c_mutex;

bool VERBOSE_OUT = false;

using namespace std;

// Define I2C Buses
string I2C_BUS_1 = "/dev/i2c-1";
string I2C_BUS_7 = "/dev/i2c-7";
string SENSOR_TOPIC = "tcp://*:5558";
string SENSOR_TOPIC_NAME = "sensor_out";

AggregateSensorData g_sensor_data;
ADS1115 pressureSensor (I2C_BUS_7, 0x48);
MPU6050 accelerometerSensor (0x68, I2C_BUS_7);
MAX30102 heartbeatSensor (I2C_BUS_7);
MAX30102 heartbeatSensor2 (I2C_BUS_1);

void setupSensors() {
    accelerometerSensor.calibrate(500);
    if (!pressureSensor.initialize() || !heartbeatSensor.initialize() || !heartbeatSensor2.initialize()) {
        if(VERBOSE_OUT){
            std::lock_guard<std::mutex> lock(cout_mutex);
            cout << "Error initializing sensors!" << endl;
        }
        exit(1);
    }
    if(VERBOSE_OUT){
        std::lock_guard<std::mutex> lock(cout_mutex);
        cout << "Sensors initialized successfully." << endl;
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
            std::lock_guard<std::mutex> lock(g_data_mutex);
            g_sensor_data.pressure_left = adc_value_left;
            g_sensor_data.pressure_right = adc_value_right;
        }
        
        {
            if(VERBOSE_OUT){
                std::lock_guard<std::mutex> lock(cout_mutex);
                cout << "Pressure Sensor Readings - Left: " << adc_value_left << " Right: " << adc_value_right << endl;
        
            }
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
            std::lock_guard<std::mutex> lock(g_data_mutex);
            g_sensor_data.roll = roll;
            g_sensor_data.pitch = pitch;
            g_sensor_data.yaw = yaw;
            g_sensor_data.accX = accX;
            g_sensor_data.accY = accY;
            g_sensor_data.accZ = accZ;
            g_sensor_data.gyroX = gyroX;
            g_sensor_data.gyroY = gyroY;
            g_sensor_data.gyroZ = gyroZ;
            
        }
        {
            if(VERBOSE_OUT){
                std::lock_guard<std::mutex> lock(cout_mutex);
                cout << "IMU Sensor Readings - Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << endl;
                cout << "Accelerometer - X: " << accX << " Y: " << accY << " Z: " << accZ << endl;
                cout << "Gyroscope - X: " << gyroX << " Y: " << gyroY << " Z: " << gyroZ << endl;
        
            }
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
            std::lock_guard<std::mutex> lock(g_data_mutex);
            g_sensor_data.hr1_heartRate = data.heartRate;
            g_sensor_data.hr1_isFingerPresent = data.isFingerPresent;
            g_sensor_data.hr1_spO2 = data.spO2;
        }
        {
            if(VERBOSE_OUT){
                std::lock_guard<std::mutex> lock(cout_mutex);
                if (readSuccess) {
                    cout << "Heart Rate Sensor Readings - Heart Rate: " << setw(8) << data.heartRate
                    << " BPM, SpO2: " << setw(8) << data.spO2 << "% , Finger Present: "
                    << (data.isFingerPresent ? "Yes" : "No") << endl;
                } else {
                    cout << "Failed to read from Heart Rate Sensor." << endl;
                }
            }
        }
        this_thread::sleep_for(chrono::milliseconds(10)); // Adjust the sleep duration as needed
    }
}

void heartrateTask2(){
    SensorData data = {0};
    while (true) {
        
        bool readSuccess;
        {
            readSuccess = heartbeatSensor2.readSensor(data);
        }
        {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            g_sensor_data.hr2_heartRate = data.heartRate;
            g_sensor_data.hr2_isFingerPresent = data.isFingerPresent;
            g_sensor_data.hr2_spO2 = data.spO2;
        }
        {
            if (VERBOSE_OUT){
                std::lock_guard<std::mutex> lock(cout_mutex);
                if (readSuccess) {
                    cout << "Heart Rate Sensor Readings - Heart Rate: " << setw(8) << data.heartRate
                    << " BPM, SpO2: " << setw(8) << data.spO2 << "% , Finger Present: "
                    << (data.isFingerPresent ? "Yes" : "No") << endl;
                } else {
                    cout << "Failed to read from Heart Rate Sensor." << endl;
                }
            }
        }
        this_thread::sleep_for(chrono::milliseconds(10)); // Adjust the sleep duration as needed
    }
}

json createJsonObject() {
    // Safely read the aggregated data
    AggregateSensorData current_data;
    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        current_data = g_sensor_data;
    }

    // Get timestamp for the entry
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    json data_entry = {
        {"timestamp_ms", timestamp},
        {"imu", {
            {"roll", current_data.roll},
            {"pitch", current_data.pitch},
            {"yaw", current_data.yaw},
            {"acceleration", {
                {"x", current_data.accX},
                {"y", current_data.accY},
                {"z", current_data.accZ}
            }},
            {"gyroscope", {
                {"x", current_data.gyroX},
                {"y", current_data.gyroY},
                {"z", current_data.gyroZ}
            }}
        }},
        {"heartrate_sensor_1", {
            {"heartRate", current_data.hr1_heartRate},
            {"spO2", current_data.hr1_spO2},
            {"fingerPresent", current_data.hr1_isFingerPresent}
        }},
        {"heartrate_sensor_2", {
            {"heartRate", current_data.hr2_heartRate},
            {"spO2", current_data.hr2_spO2},
            {"fingerPresent", current_data.hr2_isFingerPresent}
        }},
        {"pressure_sensor_adc", {
            {"left", current_data.pressure_left},
            {"right", current_data.pressure_right}
        }}
    };

    return data_entry;
}


void printAndPublishDataTask() {
    zmq::context_t context;
    zmq::socket_t socket(context, ZMQ_PUB);
    socket.bind(SENSOR_TOPIC);
    std::cout << "Established ZMQ server on " << SENSOR_TOPIC << " for sensor data" << std::endl;

    while (true) {
        json data_to_publish = createJsonObject();
        socket.send(zmq::buffer(SENSOR_TOPIC_NAME), zmq::send_flags::sndmore);
        socket.send(zmq::buffer(data_to_publish.dump()), zmq::send_flags::none);
        std::lock_guard<std::mutex> lock(cout_mutex);
        std::cout << data_to_publish << std::endl;
        // Push data every 1 second
        this_thread::sleep_for(chrono::milliseconds(1000));
    }
}


int main() {
    VERBOSE_OUT = false;

    setupSensors();

    thread pressureSensorRead(pressureSensorTask);
    thread imuSensorRead(imuSensorTask);
    thread heartrateSensorRead(heartrateTask);
    thread heartrateSensorRead2(heartrateTask2);
    thread printandPubData(printAndPublishDataTask);


    printandPubData.join();
    pressureSensorRead.join();
    imuSensorRead.join();
    heartrateSensorRead.join();
    heartrateSensorRead2.join();


    return 0;
}