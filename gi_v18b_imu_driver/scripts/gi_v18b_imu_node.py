#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import struct
from sensor_msgs.msg import Imu
from std_msgs.msg import Header, Float32
from tf.transformations import quaternion_from_euler

class GIV18BNaviTestNode:
    def __init__(self):
        rospy.init_node('gi_v18b_navitest_node')

        # Params
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud = rospy.get_param("~baudrate", 115200)

        # Serial init
        self.ser = serial.Serial(port, baud, timeout=1)

        # Publishers
        self.pub_imu = rospy.Publisher("/imu/data_raw", Imu, queue_size=10)
        self.pub_heading = rospy.Publisher("/imu/heading", Float32, queue_size=10)

        rospy.loginfo("📡 GI-V18B NAVITEST node started on %s", port)
        self.read_loop()

    def read_loop(self):
        while not rospy.is_shutdown():
            # Header 0x55 0xAA
            header = self.ser.read(2)
            if header != b'\x55\xAA':
                continue

            # Data length
            length_byte = self.ser.read(1)
            if len(length_byte) != 1 or ord(length_byte) != 0x4C:
                continue

            # Payload and checksum
            payload = self.ser.read(76)
            checksum = self.ser.read(1)

            if len(payload) != 76 or len(checksum) != 1:
                continue

            # Verify checksum
            total = sum(bytearray(length_byte + payload)) & 0xFF
            if total != ord(checksum):
                rospy.logwarn("⚠️ Checksum mismatch")
                continue

            try:
                # Unpack theo tài liệu (80 byte)
                # "<" = little endian
                # 2 x Unsigned Int, 7 x Float, 3 x Float, 1 x Float, 2 x Int, 3 x Float, 1 x Uint, 1 x ushort, 1 x short
                data = struct.unpack("<I I ffffff fff f i i fff I H h", payload)
            except struct.error:
                rospy.logwarn("⚠️ Unpack failed")
                continue

            # Tạo msg Imu
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "imu_link"

            # Gyro (angular velocity, °/s → rad/s)
            imu_msg.angular_velocity.x = data[2] * 3.1416 / 180.0
            imu_msg.angular_velocity.y = data[3] * 3.1416 / 180.0
            imu_msg.angular_velocity.z = data[4] * 3.1416 / 180.0

            # Acceleration (m/s²)
            imu_msg.linear_acceleration.x = data[5]
            imu_msg.linear_acceleration.y = data[6]
            imu_msg.linear_acceleration.z = data[7]

            # Orientation (roll, pitch, heading → quaternion)
            roll  = data[13] * 3.1416 / 180.0  # byte 60~63
            pitch = data[14] * 3.1416 / 180.0  # byte 64~67
            yaw   = data[15] * 3.1416 / 180.0  # byte 68~71 (heading)
            q = quaternion_from_euler(roll, pitch, yaw)
            imu_msg.orientation.x = q[0]
            imu_msg.orientation.y = q[1]
            imu_msg.orientation.z = q[2]
            imu_msg.orientation.w = q[3]

            # Publish IMU
            self.pub_imu.publish(imu_msg)

            # Publish heading riêng (độ)
            heading_msg = Float32()
            heading_msg.data = data[15]
            self.pub_heading.publish(heading_msg)
            # Epoch milliseconds (from 1970-01-01 00:00:00)
            # utc_seconds = data[17]      # từ byte 72~75
            # utc_millis = data[18]       # từ byte 76~77

            # utc_time = utc_seconds + (utc_millis / 1000.0)
            # print(utc_time)

if __name__ == "__main__":
    try:
        GIV18BNaviTestNode()
    except rospy.ROSInterruptException:
        pass
