#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
import sys
import threading
from ctypes import *
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import Header
from livox_ros_driver.msg import CustomMsg
import numpy as np
import cv2
import os

# Thêm Hikrobot SDK
sys.path.append('/opt/MVS/Samples/aarch64/Python/MvImport')
from MvCameraControl_class import *

class HikrobotCameraNode:
    def __init__(self):
        rospy.init_node('hikrobot_camera_node')

        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        self.info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)

        self.prev_stamp = rospy.Time.now()
        self.last_lidar_stamp = None  # Lưu timestamp mới nhất của LiDAR

        self.device_ip = rospy.get_param('~camera_ip', '192.168.0.10')
        self.host_ip = rospy.get_param('~host_ip', '192.168.0.100')

        # Sub LiDAR
        rospy.Subscriber('/livox/lidar', CustomMsg, self.lidar_callback)

        self.cam = MvCamera()
        self.init_camera()

        self.grab_thread = threading.Thread(target=self.grab_loop)
        self.grab_thread.start()

    def lidar_callback(self, msg):
        """Lưu timestamp của LiDAR để gán vào ảnh."""
        self.last_lidar_stamp = msg.header.stamp

    def init_camera(self):
        ip_list = [int(x) for x in self.device_ip.split('.')]
        net_list = [int(x) for x in self.host_ip.split('.')]

        stGigEDev = MV_GIGE_DEVICE_INFO()
        stGigEDev.nCurrentIp = (ip_list[0] << 24) | (ip_list[1] << 16) | (ip_list[2] << 8) | ip_list[3]
        stGigEDev.nNetExport = (net_list[0] << 24) | (net_list[1] << 16) | (net_list[2] << 8) | net_list[3]

        stDevInfo = MV_CC_DEVICE_INFO()
        stDevInfo.nTLayerType = MV_GIGE_DEVICE
        stDevInfo.SpecialInfo.stGigEInfo = stGigEDev

        ret = self.cam.MV_CC_CreateHandle(stDevInfo)
        if ret != 0:
            rospy.logerr("❌ Create handle failed: 0x%x" % ret)
            sys.exit()

        ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            rospy.logerr("❌ Open device failed: 0x%x" % ret)
            sys.exit()

        nPacketSize = self.cam.MV_CC_GetOptimalPacketSize()
        if nPacketSize > 0:
            self.cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)

        # Chuyển sang BayerRG8 (1 byte/pixel)
        self.cam.MV_CC_SetEnumValue("ImageCompressionMode", 0)  # tắt nén
        self.cam.MV_CC_SetEnumValue("HighBandwidthMode", 1)
        self.cam.MV_CC_SetEnumValue("PixelFormat",0x01080009)  # BayerRG8

        self.cam.MV_CC_SetIntValue("Width", 2448)
        self.cam.MV_CC_SetIntValue("Height", 2048)

        self.cam.MV_CC_SetEnumValue("ExposureAuto", 2)
        self.cam.MV_CC_SetEnumValue("GainAuto", 1)
        self.cam.MV_CC_SetEnumValue("AcquisitionFrameRateEnable", 1)
        self.cam.MV_CC_SetFloatValue("AcquisitionFrameRate", 40.0)

        val = MVCC_FLOATVALUE()
        self.cam.MV_CC_GetFloatValue("ExposureTime", val)
        rospy.loginfo("📷 ExposureTime: %.2f µs" % val.fCurValue)
        self.cam.MV_CC_GetFloatValue("Gain", val)
        rospy.loginfo("🎚️ Gain: %.2f dB" % val.fCurValue)
        self.cam.MV_CC_GetFloatValue("ResultingFrameRate", val)
        rospy.loginfo("📈 Actual FPS: %.2f" % val.fCurValue)

        ret = self.cam.MV_CC_StartGrabbing()
        if ret != 0:
            rospy.logerr("❌ Start grabbing failed: 0x%x" % ret)
            sys.exit()

        rospy.loginfo("✅ Camera started grabbing")

    def grab_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            stOutFrame = MV_FRAME_OUT()
            memset(byref(stOutFrame), 0, sizeof(stOutFrame))

            ret = self.cam.MV_CC_GetImageBuffer(stOutFrame, 100)
            if ret == 0:
                width = stOutFrame.stFrameInfo.nWidth
                height = stOutFrame.stFrameInfo.nHeight
                frame_len = stOutFrame.stFrameInfo.nFrameLen

                data = string_at(stOutFrame.pBufAddr, frame_len)

                # BayerRG8 → 1 kênh → reshape
                img_bayer = np.frombuffer(data, dtype=np.uint8).reshape((height, width))

                # Chuyển sang RGB
                img_rgb = cv2.cvtColor(img_bayer, cv2.COLOR_BayerRG2BGR)
                # alpha = 1.0  # giữ nguyên tương phản
                # beta = 50    # tăng sáng thêm 50 đơn vị

                # img_rgb = cv2.convertScaleAbs(img_rgb, alpha=alpha, beta=beta)
                # Xoay ảnh nếu cần
                img_rgb = cv2.rotate(img_rgb, cv2.ROTATE_180)

                # Gửi ROS Image
                ros_img = Image()
                ros_img.header = Header()
                # Nếu có timestamp LiDAR thì dùng, không thì dùng thời gian hiện tại
                ros_img.header.stamp = self.last_lidar_stamp if self.last_lidar_stamp else rospy.Time.now()
                ros_img.header.frame_id = "hikrobot_camera"
                ros_img.height = height
                ros_img.width = width
                ros_img.encoding = "rgb8"
                ros_img.is_bigendian = 0
                ros_img.step = width * 3
                ros_img.data = img_rgb.tobytes()
                self.image_pub.publish(ros_img)

                rospy.loginfo_throttle(1, "📸 Publishing image at %dx%d with stamp %s" % 
                                       (width, height, str(ros_img.header.stamp.to_sec())))

                self.cam.MV_CC_FreeImageBuffer(stOutFrame)
            else:
                rospy.logwarn_throttle(2, "⚠️ No image received: 0x%x" % ret)

            rate.sleep()

    def shutdown(self):
        rospy.loginfo("🛑 Shutting down camera...")
        self.cam.MV_CC_StopGrabbing()
        self.cam.MV_CC_CloseDevice()
        self.cam.MV_CC_DestroyHandle()
        rospy.loginfo("✅ Shutdown complete.")

if __name__ == '__main__':
    try:
        node = HikrobotCameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        node.shutdown()
