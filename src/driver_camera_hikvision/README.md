# Driver Camera Hikvision ROS Package

This package provides a ROS driver for Hikvision cameras, utilizing the Hikrobot SDK to capture image data and publish it to ROS topics. It also supports synchronization with LiDAR timestamps for multi-sensor applications.

# Features

- Captures raw image data from a Hikvision camera using the Hikrobot SDK.
- Publishes images to the `/camera/image_raw` topic as `sensor_msgs/Image`.
- Publishes camera calibration information to the `/camera/camera_info` topic as `sensor_msgs/CameraInfo`.
- Synchronizes image timestamps with LiDAR data from the `/livox/lidar` topic (`livox_ros_driver/CustomMsg`).
- Configurable camera settings (e.g., resolution, exposure, gain, frame rate).
- Supports BayerRG8 pixel format, converted to RGB for publishing.

# Requirements

- Ubuntu 20.04
- ROS Noetic
- Hikrobot SDK (`/opt/MVS/Samples/aarch64/Python/MvImport`)
- Required ROS packages:
  ```bash
  sudo apt-get install ros-noetic-rosserial-python ros-noetic-tf ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-livox-ros-driver
  ```
- Dependencies:
  - Python 3.8+
  - OpenCV (included with `cv_bridge`)
  - NumPy
- Hardware: Hikvision camera with GigE interface.

# Setup

## Hardware Setup
1. Connect the Hikvision camera to the network via a GigE interface.
2. Configure the camera's IP address (default: `192.168.0.10`) and ensure the host machine's IP (default: `192.168.0.100`) is on the same subnet.
3. Verify network connectivity:
   ```bash
   ping 192.168.0.10
   ```

## Software Setup
1. Install the Hikrobot SDK:
   - Download and install the MVS software from the Hikvision website.
   - Ensure the Python SDK is available at `/opt/MVS/Samples/aarch64/Python/MvImport`.
2. Build the ROS workspace:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

# Usage

## Launch the Driver
Run the camera driver:
```bash
roslaunch driver_camera_hikvision camera.launch
```

This will start:
- **Node**: `hikvision_camera_node` (from `camera_image.py`)
- **Published Topics**:
  - `/camera/image_raw` (`sensor_msgs/Image`): Raw RGB images from the camera.
  - `/camera/camera_info` (`sensor_msgs/CameraInfo`): Camera calibration information.
- **Subscribed Topic**:
  - `/livox/lidar` (`livox_ros_driver/CustomMsg`): LiDAR data for timestamp synchronization.
- **Parameters**:
  - `~camera_ip` (default: `192.168.0.10`): IP address of the Hikvision camera.
  - `~host_ip` (default: `192.168.0.100`): IP address of the host machine.

## Launch File Configuration
The provided launch file (`hikvision_camera.launch`) includes:
- Main node: `camera_image.py` for publishing image data.
- Commented nodes (optional):
  - `pe_calibration_publisher.py`: For publishing calibration data.
  - `pub_info.py`: For publishing additional camera information.

To enable optional nodes, uncomment the corresponding lines in `hikvision_camera.launch`.

# Example

1. Check published images:
   ```bash
   rosrun image_view image_view image:=/camera/image_raw
   ```
2. Check camera info:
   ```bash
   rostopic echo /camera/camera_info
   ```
3. Visualize in RViz:
   ```bash
   rosrun rviz rviz
   ```

# Troubleshooting

| Issue                    | Cause                          | Solution                                                                 |
|--------------------------|--------------------------------|--------------------------------------------------------------------------|
| No image data            | Incorrect camera IP           | Verify `~camera_ip` and `~host_ip` in the launch file                     |
| Failed to open camera    | SDK or network issue          | Ensure Hikrobot SDK is installed and camera is reachable via `ping`       |
| Timestamp mismatch       | LiDAR topic not published     | Ensure `/livox/lidar` topic is active                                     |
| Low frame rate           | Network bandwidth or settings | Check camera settings (e.g., `AcquisitionFrameRate`) and network          |
| Node crashes             | Missing dependencies          | Install required ROS packages and ensure SDK path is correct              |

# Notes

- Ensure the Hikvision camera is configured for GigE and the SDK is properly installed at `/opt/MVS/Samples/aarch64/Python/MvImport`.
- The driver uses BayerRG8 format and converts to RGB. Adjust the pixel format in `camera_image.py` if your camera uses a different format.
- The camera resolution is set to 2448x2048 by default. Modify `Width` and `Height` in `camera_image.py` to match your camera's specifications.
- The driver synchronizes image timestamps with LiDAR data if available; otherwise, it uses the current ROS time.
- To use optional nodes (`pe_calibration_publisher.py` or `pub_info.py`), implement and uncomment them in the launch file.