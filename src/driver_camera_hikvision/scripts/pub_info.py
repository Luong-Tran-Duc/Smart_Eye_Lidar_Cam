#!/usr/bin/env python

import rospy
import yaml
from sensor_msgs.msg import CameraInfo

def load_camera_info(yaml_file):
    with open(yaml_file, 'r') as f:
        calib = yaml.safe_load(f)

    cam_info = CameraInfo()
    cam_info.width = calib['image_width']
    cam_info.height = calib['image_height']
    cam_info.distortion_model = calib['distortion_model']
    cam_info.D = calib['distortion_coefficients']['data']
    cam_info.K = calib['camera_matrix']['data']
    cam_info.R = calib['rectification_matrix']['data']
    cam_info.P = calib['projection_matrix']['data']
    cam_info.header.frame_id = calib.get('camera_name', 'camera')

    return cam_info

def publisher_loop(yaml_path, topic_name, rate_hz):
    rospy.init_node('camera_info_publisher', anonymous=True)
    pub = rospy.Publisher(topic_name, CameraInfo, queue_size=10)

    cam_info = load_camera_info(yaml_path)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        cam_info.header.stamp = rospy.Time.now()
        pub.publish(cam_info)
        rate.sleep()

if __name__ == "__main__":
    # ⚠️ THAY ĐỔI GIÁ TRỊ TẠI ĐÂY
    yaml_path = "iniit.yaml"
    topic_name = "/camera/camera_info"
    rate_hz = 10

    try:
        publisher_loop(yaml_path, topic_name, rate_hz)
    except rospy.ROSInterruptException:
        pass
