;; Auto-generated. Do not edit!


(when (boundp 'common::ScanMapStatus)
  (if (not (find-package "COMMON"))
    (make-package "COMMON"))
  (shadow 'ScanMapStatus (find-package "COMMON")))
(unless (find-package "COMMON::SCANMAPSTATUS")
  (make-package "COMMON::SCANMAPSTATUS"))

(in-package "ROS")
;;//! \htmlinclude ScanMapStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass common::ScanMapStatus
  :super ros::object
  :slots (_header _scan_status _file_name ))

(defmethod common::ScanMapStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:scan_status __scan_status) "")
    ((:file_name __file_name) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _scan_status (string __scan_status))
   (setq _file_name (string __file_name))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:scan_status
   (&optional __scan_status)
   (if __scan_status (setq _scan_status __scan_status)) _scan_status)
  (:file_name
   (&optional __file_name)
   (if __file_name (setq _file_name __file_name)) _file_name)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _scan_status
    4 (length _scan_status)
    ;; string _file_name
    4 (length _file_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _scan_status
       (write-long (length _scan_status) s) (princ _scan_status s)
     ;; string _file_name
       (write-long (length _file_name) s) (princ _file_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _scan_status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _scan_status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _file_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _file_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get common::ScanMapStatus :md5sum-) "99a910d59af2cd6d4413707fa4ffd257")
(setf (get common::ScanMapStatus :datatype-) "common/ScanMapStatus")
(setf (get common::ScanMapStatus :definition-)
      "# ScanMapStatus.msg

# Standard ROS message header
#   stamp: time of status update
#   frame_id: optional, can be empty
std_msgs/Header header

# Scan status
# Valid values: idle, scanning, completed, failed
string scan_status

# File name generated after scan is completed (if any)
string file_name
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :common/ScanMapStatus "99a910d59af2cd6d4413707fa4ffd257")


