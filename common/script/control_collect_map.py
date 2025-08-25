#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool, Header
from common.msg import ScanMapStatus
import subprocess
import signal
import threading
import time
import glob
import os

launch_process = None
pub = None
seq_counter = 0

# Biến lưu trạng thái hiện tại
current_status = "idle"
current_file_name = ""

def publish_status(status, file_name=""):
    """Publish ScanMapStatus message."""
    global seq_counter
    msg = ScanMapStatus()
    header = Header()
    header.seq = seq_counter
    header.stamp = rospy.Time.now()
    header.frame_id = "map_status"
    seq_counter += 1

    msg.header = header
    msg.scan_status = status
    msg.file_name = file_name
    pub.publish(msg)
    rospy.loginfo(f"Published scan_status='{status}', file='{file_name}'")

def get_latest_file(directory):
    """Return latest file in directory or empty string."""
    files = glob.glob(os.path.join(directory, "*"))
    if not files:
        return ""
    return max(files, key=os.path.getmtime)

def handle_start():
    global current_status, current_file_name
    time.sleep(10)
    current_status = "scanning"
    current_file_name = ""

def handle_stop():
    global current_status, current_file_name
    time.sleep(10)
    latest_file = get_latest_file("/home/viact/Data_scan/LAS")
    current_status = "completed"
    current_file_name = os.path.basename(latest_file)
    time.sleep(10)
    current_status = "idle"
    

def callback(msg):
    global launch_process, current_status
    if msg.data:  # True → Start
        if launch_process is None:
            rospy.loginfo("Starting launch file...")
            launch_process = subprocess.Popen(
                ["roslaunch", "fast_livo", "mapping_avia.launch"]
            )
            current_status = "starting"
            threading.Thread(target=handle_start, daemon=True).start()
        else:
            rospy.logwarn("Launch file is already running.")
    else:  # False → Stop
        if launch_process is not None:
            rospy.loginfo("Stopping launch file...")
            launch_process.send_signal(signal.SIGINT)  # Ctrl+C
            launch_process.wait()
            launch_process = None
            current_status = "stopping"
            threading.Thread(target=handle_stop, daemon=True).start()
        else:
            rospy.logwarn("No launch file to stop.")

def status_publisher_loop():
    """Publish trạng thái liên tục mỗi 1 giây."""
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        publish_status(current_status, current_file_name)
        rate.sleep()

def listener():
    global pub
    rospy.init_node('launch_controller')
    pub = rospy.Publisher("/map/status", ScanMapStatus, queue_size=10)
    rospy.Subscriber("/map/command_scan", Bool, callback)
    rospy.loginfo("Listening on /control_launch (Bool)...")

    # Chạy thread publish liên tục
    threading.Thread(target=status_publisher_loop, daemon=True).start()

    rospy.spin()

if __name__ == "__main__":
    listener()
