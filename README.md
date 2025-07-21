# Vectornav VN100 IMU

## Overview
This project provides a C++ implementation for interfacing with the VectorNav VN100 Inertial Measurement Unit (IMU). It allows for configuration, connection, data reading, and processing of sensor data (yaw, pitch, roll, angular rates, quaternions, and acceleration) from the VN100 sensor.

## Directory Structure
```
vec/
├── include/
│   └── vn100.h
├── src/
│   ├── main.cpp
│   └── vn100.cpp
├── libraries/
│   └── vectornav/
├── CMakelists.txt
└── config.cfg
```

## Prerequisites
- **C++ Compiler**: Requires C++17 or later.
- **CMake**: Version 3.10 or higher for building the project.
- **VectorNav Library**: The VectorNav library must be located in the `libraries/vectornav/` directory. Ensure the library's `CMakeLists.txt` is present in `libraries/vectornav/`.
- **Hardware**: A VectorNav VN100 IMU connected via a serial port (e.g., `/dev/cu.usbserial-AI06JFNW`).

## Installation
1. **Clone the Repository**:
   ```bash
   git clone <repository_url>
   cd vec
   ```

2. **Ensure VectorNav Library**:
   - Place the VectorNav library in the `libraries/vectornav/` directory.
   - Verify that `libraries/vectornav/CMakeLists.txt` exists.

3. **Build the Project**:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

4. **Output**:
   - The executable `vn100_imu` will be generated in the `build` directory.

## Configuration
The `config.cfg` file in the `vec/` directory specifies the VN100 sensor settings:
- **port**: Serial port for the VN100 (e.g., `/dev/cu.usbserial-AI06JFGW`).
- **baud_rate**: Communication baud rate (e.g., `230400`).
- **frequency**: Data output frequency in Hz (e.g., `200`).

Example `config.cfg`:
```
vn100:
 on: 1
 port: "/dev/cu.usbserial-AI06JFGW"
 baud_rate: 230400
 frequency: 200
```

Modify these parameters as needed for your setup.

## Usage
1. **Run the Program**:
   ```bash
   ./build/vn100_imu
   ```

2. **Program Behavior**:
   - Loads configuration from `config.cfg`.
   - Connects to the VN100 sensor using the specified port and baud rate.
   - Reads sensor data (yaw, pitch, roll, angular rates, quaternions, and acceleration) at the configured frequency.
   - Outputs data to the console for 10 seconds.
   - Closes the connection to the sensor.

3. **Output Format**:
   - The program prints sensor data in the format:
     ```
     YPR: [yaw, pitch, roll] | Gyro: [x, y, z] | Accel: [x, y, z] | Quaternion: [w, x, y, z]
     ```

## Key Files
- **vec/include/vn100.h**: Header file defining the `vn100` class for interfacing with the VN100 sensor.
- **vec/src/vn100.cpp**: Implementation of the `vn100` class, handling configuration, connection, and data retrieval.
- **vec/src/main.cpp**: Main program that demonstrates usage of the `vn100` class.
- **vec/CMakelists.txt**: CMake configuration for building the project.
- **vec/config.cfg**: Configuration file for VN100 sensor settings.

## Notes
- Ensure the serial port and baud rate match your VN100 sensor's configuration.
- The program runs for a fixed duration (10 seconds). Modify the `cycles` variable in `main.cpp` to adjust the runtime.
- Error handling is implemented to catch and display issues with configuration, connection, or data retrieval.
- The VectorNav library must be properly installed in `libraries/vectornav/` for the project to build successfully.

## Troubleshooting
- **Configuration File Errors**: Verify that `config.cfg` exists and contains valid `port`, `baud_rate`, and `frequency` entries.
- **Connection Issues**: Ensure the VN100 sensor is connected and the specified port is correct.
- **Library Not Found**: Confirm that the VectorNav library is placed in `libraries/vectornav/` with the required `CMakeLists.txt`.
- **Build Errors**: Check for missing dependencies or incorrect paths in `CMakelists.txt`.