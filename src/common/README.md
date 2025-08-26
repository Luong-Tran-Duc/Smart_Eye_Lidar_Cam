# Multi-Sensor ROS Package

This ROS package integrates multiple sensor drivers (camera, GI-V18B IMU, and Livox LiDAR) and control nodes for data collection and synchronization, designed to work with ROS Noetic on Ubuntu 20.04.

# Features

- Launches a camera driver for capturing image data.
- Integrates the GI-V18B IMU driver for inertial measurements (gyro, accel, orientation).
- Runs the Livox LiDAR driver with RViz support for point cloud visualization.
- Includes control nodes for map data collection and MCU synchronization.

# Requirements

- Ubuntu 20.04
- ROS Noetic
- Required ROS packages:
  ```bash
  sudo apt-get install ros-noetic-rosserial-python ros-noetic-tf
  ```
- Hardware:
  - Camera connected to the system.
  - GI-V18B IMU connected via USB-to-serial converter (default: `/dev/ttyUSB0`).
  - Livox LiDAR connected to the system.

# Usage

## Hardware Setup
1. Connect the camera to the system.
2. Connect the GI-V18B IMU via a USB-to-serial converter. Verify the port:
   ```bash
   ls /dev/ttyUSB*
   ```
3. Connect the Livox LiDAR to the system.

## Launch the System
Run the launch file to start all sensor drivers and control nodes:
```bash
roslaunch common all.launch
```

This will:
- Launch the camera driver (`camera_driver/launch/camera.launch`).
- Launch the GI-V18B IMU driver (`gi_v18b_imu_driver/launch/gi_v18b_imu.launch`).
- Launch the Livox LiDAR driver with RViz (`livox_ros_driver/launch/livox_lidar_msg.launch`).
- Start the control nodes:
  - `control_collect_map.py`: Collects map data.
  - `pub_sync_mcu.py`: Publishes synchronization data to the MCU.

# Published Topics

Refer to individual sensor driver documentation for specific topics:
- **Camera**: Check `camera_driver` package for image topics.
- **GI-V18B IMU**:
  - `/imu/data_raw` (`sensor_msgs/Imu`): Raw IMU data (gyro, accel, quaternion).
  - `/imu/heading` (`std_msgs/Float32`): Heading (yaw) in degrees.
- **Livox LiDAR**: Check `livox_ros_driver` package for point cloud topics.

# Coordinate System (GI-V18B IMU)

- **Gyroscope**: Converted from °/s to rad/s.
- **Acceleration**: Expressed in m/s².
- **Orientation**: Converted to quaternion (roll, pitch, yaw).
- **Yaw (heading)**: Published separately as `/imu/heading`.

# Example

1. Check IMU data:
   ```bash
   rostopic echo /imu/data_raw
   ```
2. Check heading:
   ```bash
   rostopic echo /imu/heading
   ```
3. Visualize LiDAR data in RViz:
   ```bash
   rosrun rviz rviz
   ```

# Troubleshooting

| Issue                    | Cause                          | Solution                                                                 |
|--------------------------|--------------------------------|--------------------------------------------------------------------------|
| No IMU data              | Wrong serial port             | Update `port:=/dev/ttyUSBX` in `gi_v18b_imu.launch`                      |
| IMU checksum mismatch    | Noise or baud mismatch        | Ensure `baudrate=115200`, check IMU cable connections                    |
| Permission denied        | User not in dialout group     | Run: `sudo usermod -a -G dialout $USER && reboot`                        |
| No LiDAR data            | Incorrect Livox connection     | Verify Livox LiDAR connection and configuration                          |
| No camera data           | Camera not connected          | Check camera connection and `camera.launch` configuration                |
| Control nodes not running| Missing dependencies          | Ensure `common` package is built and sourced                              |

# Notes

- Ensure all required packages (`camera_driver`, `gi_v18b_imu_driver`, `livox_ros_driver`, `common`) are properly installed and sourced in your ROS workspace.
- Replace `<package_name>` with the actual name of your ROS package containing the `main.launch` file.
- For detailed sensor-specific configurations, refer to the respective driver packages.