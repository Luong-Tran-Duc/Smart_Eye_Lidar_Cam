#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from datetime import datetime

class ImageStreamer:
    def __init__(self):
        self.fps = 15
        self.width = 2048
        self.height = 2048

        self.bridge = CvBridge()
        self.last_time = rospy.Time.now()

        self.out = cv2.VideoWriter(
            'appsrc ! videoconvert ! video/x-raw,format=I420' +
            ' ! x264enc speed-preset=ultrafast tune=zerolatency key-int-max=' + str(self.fps * 2) +
            ' ! video/x-h264,profile=baseline' +
            ' ! rtspclientsink location=rtsp://localhost:8554/mystream',
            cv2.CAP_GSTREAMER,
            0,
            self.fps,
            (self.width, self.height),
            True
        )

        if not self.out.isOpened():
            raise Exception("⚠️ Cannot open GStreamer pipeline!")

        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        rospy.loginfo("✅ Image streamer initialized and waiting for image messages...")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            rospy.logerr("Bridge error: %s", e)
            return

        # Resize to match VideoWriter if needed
        if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
            cv_image = cv2.resize(cv_image, (self.width, self.height))

        self.out.write(cv_image)
        rospy.loginfo("🟢 Frame written at %s" % datetime.now())

if __name__ == '__main__':
    rospy.init_node('rtsp_image_streamer')
    try:
        streamer = ImageStreamer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
