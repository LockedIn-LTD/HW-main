#include <MPU6050.hpp>

// Constructor now uses std::string busPath
MPU6050::MPU6050(uint8_t addr, const std::string& busPath)
    : MPU6050_addr(addr), f_dev(-1), calc_yaw(false),
      _first_run(true), _calibrated(false),
      A_OFF_X(0), A_OFF_Y(0), A_OFF_Z(0),
      G_OFF_X(0), G_OFF_Y(0), G_OFF_Z(0)
{
    // Open I2C bus using the string path
    f_dev = open(busPath.c_str(), O_RDWR);
    if (f_dev < 0) {
        std::cerr << "Failed to open I2C bus: " << busPath << std::endl;
        return;
    }
    // Set the slave address
    if (ioctl(f_dev, I2C_SLAVE, MPU6050_addr) < 0) {
        std::cerr << "Failed to acquire bus access to MPU6050" << std::endl;
        return;
    }

    // Initialize MPU6050: Wake up, set Gyro/Accel ranges
    i2c_smbus_write_byte_data(f_dev, 0x6B, 0x00); // Power Management 1: Clear SLEEP bit
    i2c_smbus_write_byte_data(f_dev, 0x1B, GYRO_CONFIG); // Gyro Config
    i2c_smbus_write_byte_data(f_dev, 0x1C, ACCEL_CONFIG); // Accel Config

    clock_gettime(CLOCK_REALTIME, &start);
}

// Internal raw read
void MPU6050::_readRaw() {
    if (f_dev < 0) return;
    
    // Read raw 16-bit values from MPU6050 registers (Accel: 0x3B-0x40, Gyro: 0x43-0x48)
    int16_t raw_ax = (i2c_smbus_read_byte_data(f_dev, 0x3B) << 8) | i2c_smbus_read_byte_data(f_dev, 0x3C);
    int16_t raw_ay = (i2c_smbus_read_byte_data(f_dev, 0x3D) << 8) | i2c_smbus_read_byte_data(f_dev, 0x3E);
    int16_t raw_az = (i2c_smbus_read_byte_data(f_dev, 0x3F) << 8) | i2c_smbus_read_byte_data(f_dev, 0x40);

    int16_t raw_gr = (i2c_smbus_read_byte_data(f_dev, 0x43) << 8) | i2c_smbus_read_byte_data(f_dev, 0x44);
    int16_t raw_gp = (i2c_smbus_read_byte_data(f_dev, 0x45) << 8) | i2c_smbus_read_byte_data(f_dev, 0x46);
    int16_t raw_gy = (i2c_smbus_read_byte_data(f_dev, 0x47) << 8) | i2c_smbus_read_byte_data(f_dev, 0x48);

    // Apply calibration offsets and convert to engineering units
    ax = (float)(raw_ax - A_OFF_X) / ACCEL_SENS;
    ay = (float)(raw_ay - A_OFF_Y) / ACCEL_SENS;
    az = (float)(raw_az - A_OFF_Z) / ACCEL_SENS; // Accelerometer data in G's

    gr = (float)(raw_gr - G_OFF_X) / GYRO_SENS;
    gp = (float)(raw_gp - G_OFF_Y) / GYRO_SENS;
    gy = (float)(raw_gy - G_OFF_Z) / GYRO_SENS; // Gyroscope data in °/s
}

// Complementary Filter Update (runs in background thread)
void MPU6050::_update() {
    if (!_calibrated || f_dev < 0) return;

    // Calculate time step (dt)
    clock_gettime(CLOCK_REALTIME, &end);
    dt = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9f;
    clock_gettime(CLOCK_REALTIME, &start);

    _readRaw(); // Get fresh sensor data

    // 1. Accelerometer Angle Calculation (Roll/Pitch)
    _accel_angle[0] = atan2(ay, az) * RAD_TO_DEG; // Roll
    _accel_angle[1] = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG; // Pitch
    if (calc_yaw)
        _accel_angle[2] = atan2(az, sqrt(ax*ax + ay*ay)) * RAD_TO_DEG;

    // 2. Initialize Gyro Angle
    if (_first_run) {
        _gyro_angle[0] = _accel_angle[0];
        _gyro_angle[1] = _accel_angle[1];
        _gyro_angle[2] = _accel_angle[2];
        _first_run = false;
    }

    // 3. Gyroscope Integration
    _gyro_angle[0] += gr * dt;
    _gyro_angle[1] += gp * dt;
    _gyro_angle[2] += gy * dt;

    // 4. Complementary Filter
    // Combine Gyro (short-term) and Accel (long-term) data
    for (int i=0; i<3; i++)
        _angle[i] = TAU*(_gyro_angle[i]) + (1-TAU)*(_accel_angle[i]);
}

// Start background update thread
void MPU6050::_startThread() {
    std::thread([this]() {
        while (true) {
            this->_update();
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // ~100Hz update rate
        }
    }).detach();
}

// Calibration function
void MPU6050::calibrate(int samples) {
    if (f_dev < 0) {
        std::cerr << "Cannot calibrate MPU6050: I2C device not open." << std::endl;
        return;
    }

    int64_t ax_sum=0, ay_sum=0, az_sum=0;
    int64_t gr_sum=0, gp_sum=0, gy_sum=0;

    for (int i=0; i<samples; i++) {
        // Read raw sensor data
        int16_t raw_ax = (i2c_smbus_read_byte_data(f_dev, 0x3B)<<8)|i2c_smbus_read_byte_data(f_dev,0x3C);
        int16_t raw_ay = (i2c_smbus_read_byte_data(f_dev, 0x3D)<<8)|i2c_smbus_read_byte_data(f_dev,0x3E);
        int16_t raw_az = (i2c_smbus_read_byte_data(f_dev, 0x3F)<<8)|i2c_smbus_read_byte_data(f_dev,0x40);

        int16_t raw_gr = (i2c_smbus_read_byte_data(f_dev,0x43)<<8)|i2c_smbus_read_byte_data(f_dev,0x44);
        int16_t raw_gp = (i2c_smbus_read_byte_data(f_dev,0x45)<<8)|i2c_smbus_read_byte_data(f_dev,0x46);
        int16_t raw_gy = (i2c_smbus_read_byte_data(f_dev,0x47)<<8)|i2c_smbus_read_byte_data(f_dev,0x48);

        ax_sum += raw_ax; ay_sum += raw_ay; az_sum += raw_az;
        gr_sum += raw_gr; gp_sum += raw_gp; gy_sum += raw_gy;
        usleep(1000); // 1ms delay
    }

    // Calculate Accelerometer Offsets: Subtract 1G (ACCEL_SENS) from Z-axis reading
    A_OFF_X = (float)ax_sum / samples;
    A_OFF_Y = (float)ay_sum / samples;
    A_OFF_Z = (float)az_sum / samples - ACCEL_SENS; 
    
    // Calculate Gyroscope Offsets
    G_OFF_X = (float)gr_sum / samples;
    G_OFF_Y = (float)gp_sum / samples;
    G_OFF_Z = (float)gy_sum / samples;

    _calibrated = true;
    _startThread(); // Start the continuous update thread after calibration
}

void MPU6050::getAccel(float *x, float *y, float *z) { _readRaw(); *x = ax; *y = ay; *z = az; }
void MPU6050::getGyro(float *roll, float *pitch, float *yaw) { _readRaw(); *roll = gr; *pitch = gp; *yaw = gy; }
void MPU6050::getAccelRaw(float *x, float *y, float *z) { _readRaw(); *x = ax * ACCEL_SENS; *y = ay * ACCEL_SENS; *z = az * ACCEL_SENS; }
void MPU6050::getGyroRaw(float *roll, float *pitch, float *yaw) { _readRaw(); *roll = gr * GYRO_SENS; *pitch = gp * GYRO_SENS; *yaw = gy * GYRO_SENS; }

int MPU6050::getAngle(int axis, float *result) {
    if (axis < 0 || axis > 2) return -1;
    *result = _angle[axis];
    return 0;
}
