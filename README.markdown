# Smart Eye Lidar Camera ROS Package for Mapping

This package integrates a Hikvision camera, Livox LiDAR, and GI-V18B IMU for mapping using `FAST-LIVO2` in ROS Noetic on Ubuntu 20.04. It includes instructions for installing dependencies, setting up time synchronization, cloning repositories, building the project, and configuring a systemd service for automatic startup.

# Requirements
- **OS**: Ubuntu 20.04
- **ROS**: Noetic
- **Hardware**:
  - Hikvision camera (GigE, IP: `192.168.0.10`)
  - Livox LiDAR (Ethernet: `192.168.1.10`)
  - GI-V18B IMU (UART, e.g., `/dev/ttyUSB0`)
- **Software**:
  - Hikrobot SDK (`/opt/MVS/Samples/aarch64/Python/MvImport`)
  - Livox SDK
  - Dependencies: PCL (>=1.8), Eigen (>=3.3.4), OpenCV (>=4.2), Sophus, Ceres Solver
  - ROS packages:
    ```bash
    sudo apt install ros-noetic-rosserial-python ros-noetic-tf ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-livox-ros-driver ros-noetic-pcl-ros
    ```

# Installation

## 1. Install Dependencies

### ROS Noetic
Follow: [ROS Noetic Installation](https://wiki.ros.org/noetic/Installation/Ubuntu)
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full python3-catkin-tools python3-rosinstall python3-wstool
sudo rosdep init
rosdep update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Hikrobot SDK
1. Download: [MVS-3.0.1_aarch64_20241128.tar.gz](https://drive.google.com/file/d/1drGAChy3lFAHBDnRVjkTyGjRGW2P9YQ1/view?usp=drive_link)
2. Extract and install:
   ```bash
   tar -xzf ~/Downloads/MVS-3.0.1_aarch64_20241128.tar.gz -C ~/Downloads
   cd ~/Downloads/MVS-3.0.1_aarch64_20241128
   sudo ./install.sh
   ```
3. Configure camera:
   - Open MVS tool:
     ```bash
     /opt/MVS/bin/MVS.sh
     ```
   - Set camera IP: `192.168.0.10`
   - Set host Ethernet IP (camera): `192.168.0.100`
   - Verify: `ping 192.168.0.10`

### FAST-LIVO2 Dependencies
1. Install PCL, Eigen, OpenCV:
   ```bash
   sudo apt install libpcl-dev libeigen3-dev libopencv-dev python3-opencv
   ```
2. Install Sophus:
   ```bash
   cd ~/Downloads
   git clone https://github.com/strasdat/Sophus.git
   cd Sophus
   git checkout a621ff
   mkdir build && cd build
   cmake .. && make
   sudo make install
   ```
3. Install Ceres Solver:
   ```bash
   sudo apt install libceres-dev
   ```

### Time Synchronization (PTP)
1. Install PTP daemon:
   ```bash
   sudo apt install ptpd
   ```
2. Configure PTP:
   - Verify Ethernet interface (`eth2`):
     ```bash
     ip link show
     ```
   - Test PTP:
     ```bash
     sudo ptpd -M -i eth2
     ```

### Network Configuration
- Camera Ethernet: `192.168.0.100`
- LiDAR Ethernet: `192.168.1.10`
- Verify:
  ```bash
  ip addr show
  ping 192.168.0.10
  ping 192.168.1.10
  ```

## 2. Build Program
1. Create workspace:
   ```bash
   mkdir -p ~/Smart_Eye_Lidar_Cam/src
   cd ~/Smart_Eye_Lidar_Cam
   catkin init
   ```
2. Clone repositories:
   ```bash
   cd ~/Smart_Eye_Lidar_Cam/src
   git clone --recurse-submodules git@github.com:Luong-Tran-Duc/Smart_Eye_Lidar_Cam.git smart_eye_lidar_cam
   git clone https://github.com/hku-mars/FAST-LIVO2.git
   ```
3. Build:
   ```bash
   cd ~/Smart_Eye_Lidar_Cam
   catkin build --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
   echo "source ~/Smart_Eye_Lidar_Cam/devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## 3. Usage
### Setup Systemd Service
1. Create `/home/viact/Smart_Eye_Lidar_Cam/start_all.sh`:
   ```bash
   #!/bin/bash
   sudo ptpd -M -i eth2
   sudo chmod 777 /dev/tty*
   echo rs485 | sudo tee /sys/class/sp339_mode_ctl/uartMode > /dev/null
   roslaunch common all.launch
   ```
   Make executable:
   ```bash
   chmod +x /home/viact/Smart_Eye_Lidar_Cam/start_all.sh
   ```
2. Create `/home/viact/Smart_Eye_Lidar_Cam/smart_eye_lidar_cam.service`:
   ```ini
   [Unit]
   Description=Smart Eye Lidar Camera Startup Service
   After=network.target

   [Service]
   Type=simple
   ExecStart=/home/viact/Smart_Eye_Lidar_Cam/start_all.sh
   Environment="ROS_DISTRO=noetic"
   Environment="PATH=/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
   User=viact
   Group=viact
   Restart=on-failure
   WorkingDirectory=/home/viact/Smart_Eye_Lidar_Cam

   [Install]
   WantedBy=multi-user.target
   ```
3. Enable service:
   ```bash
   sudo cp /home/viact/Smart_Eye_Lidar_Cam/smart_eye_lidar_cam.service /etc/systemd/system/
   sudo chmod 644 /etc/systemd/system/smart_eye_lidar_cam.service
   sudo systemctl enable smart_eye_lidar_cam.service
   sudo systemctl start smart_eye_lidar_cam.service
   ```

### Manual Run (Optional)
```bash
roslaunch driver_camera_hikvision hikvision_camera.launch
roslaunch gi_v18b_imu_driver gi_v18b_imu.launch
roslaunch livox_ros_driver livox_lidar_msg.launch
roslaunch fast_livo fast_livo.launch
```

### Verify Data
```bash
rosrun image_view image_view image:=/camera/image_raw
rostopic echo /imu/data_raw
rostopic echo /livox/lidar
rosrun rviz rviz
```

# Troubleshooting
| Issue                    | Cause                          | Solution                                                                 |
|--------------------------|--------------------------------|--------------------------------------------------------------------------|
| No camera data           | IP mismatch                   | Verify camera IP (`192.168.0.10`), host IP (`192.168.0.100`)             |
| No LiDAR data            | Network issue                 | Check LiDAR IP (`192.168.1.10`), ensure `livox_ros_driver` running        |
| PTP fails                | Wrong interface               | Verify `eth2` (`ip link show`), install `ptpd` (`sudo apt install ptpd`)  |
| Service fails            | Permissions                   | Ensure `start_all.sh` executable, user `viact` in `dialout` group         |
| Build error              | CMake issue                   | Update CMake (`sudo apt install cmake`), retry with `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` |
| Submodule error          | Repository conflict           | Run: `git rm -r --cached src/smart_eye_lidar_cam` and re-clone           |

# Notes
- **Time Synchronization**: PTP (`ptpd -M -i eth2`) ensures synchronized timestamps. Verify `eth2` and device PTP support.
- **Network**: Camera (`192.168.0.10`) and LiDAR (`192.168.1.10`) must be on correct subnets.
- **UART**: Add user to `dialout` for IMU:
  ```bash
  sudo usermod -a -G dialout $USER
  reboot
  ```
- **Repository Links**:
  - Camera: [driver_camera_hikvision](https://github.com/Luong-Tran-Duc/Smart_Eye_Lidar_Cam/tree/main/src/driver_camera_hikvision)
  - LiDAR: [livox_ros_driver](https://github.com/Kham96/livox_ros_driver/tree/3d240d5666129e1a3052e78ee8487a04b08fdda3)
  - IMU: [gi_v18b_imu_driver](https://github.com/Luong-Tran-Duc/Smart_Eye_Lidar_Cam/tree/main/src/gi_v18b_imu_driver)
  - Mapping: [FAST-LIVO2](https://github.com/Luong-Tran-Duc/Smart_Eye_Lidar_Cam/tree/main/src/FAST-LIVO2)
  - Common: [common](https://github.com/Luong-Tran-Duc/Smart_Eye_Lidar_Cam/tree/main/src/common)
- **Launch File**: Ensure `common all.launch` includes nodes for `driver_camera_hikvision`, `gi_v18b_imu_driver`, `livox_ros_driver`, and `fast_livo`.
- Verify SSH access to `git@github.com:Luong-Tran-Duc/Smart_Eye_Lidar_Cam.git`.