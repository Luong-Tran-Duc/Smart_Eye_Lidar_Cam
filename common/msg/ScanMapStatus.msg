# ScanMapStatus.msg

# Standard ROS message header
#   stamp: time of status update
#   frame_id: optional, can be empty
std_msgs/Header header

# Scan status
# Valid values: idle, scanning, completed, failed
string scan_status

# File name generated after scan is completed (if any)
string file_name