# GI-V18B IMU Driver (NAVITEST Protocol)

This package provides a ROS driver for the GI-V18B IMU, parsing the NAVITEST binary protocol over UART and publishing IMU data into standard ROS topics.

# Features

- Reads raw IMU data (gyroscope, accelerometer, orientation) from GI-V18B over UART.
- Parses NAVITEST protocol (0x55 0xAA header, 80-byte frame).
- Publishes:
  - `/imu/data_raw` → `sensor_msgs/Imu`
  - `/imu/heading` → `std_msgs/Float32` (yaw angle in degrees)

# Requirements

- Ubuntu 20.04 / ROS Noetic
- Python 3.8+
- ROS packages:
  ```bash
  sudo apt-get install ros-noetic-rosserial-python ros-noetic-tf
  ```
- Serial device connected (default: `/dev/ttyUSB0`)

# Usage

## Connect IMU
Connect the GI-V18B IMU via a USB-to-serial converter.

Check available serial devices:
```bash
ls /dev/ttyUSB*
```

## Launch the Driver
Run the driver using:
```bash
roslaunch gi_v18b_imu_driver gi_v18b_imu.launch
```

This will start:
- **Node**: `gi_v18b_imu_node.py`
- **Parameters**:
  - `port` (default: `/dev/ttyUSB0`)
  - `baudrate` (default: `115200`)

# Published Topics

| Topic             | Type                   | Description                          |
|-------------------|------------------------|--------------------------------------|
| `/imu/data_raw`   | `sensor_msgs/Imu`      | Raw IMU data (gyro, accel, quaternion) |
| `/imu/heading`    | `std_msgs/Float32`     | Heading (yaw) in degrees             |

# Coordinate System

- **Gyroscope**: Converted from °/s to rad/s.
- **Acceleration**: Expressed in m/s².
- **Orientation**: Converted to quaternion (roll, pitch, yaw).
- **Yaw (heading)**: Published separately as `/imu/heading`.

# Example

Check IMU messages:
```bash
rostopic echo /imu/data_raw
```

Check heading:
```bash
rostopic echo /imu/heading
```

# Troubleshooting

| Issue                    | Cause                          | Solution                                                                 |
|--------------------------|--------------------------------|--------------------------------------------------------------------------|
| No data received         | Wrong serial port             | Update `port:=/dev/ttyUSBX` in the launch file                           |
| Checksum mismatch logs   | Noise or baud mismatch        | Ensure `baudrate=115200`, check cable connections                        |
| Orientation unstable     | Sensor not fixed properly     | Mount sensor rigidly                                                     |
| Permission denied        | User not in dialout group     | Run: `sudo usermod -a -G dialout $USER && reboot`                        |