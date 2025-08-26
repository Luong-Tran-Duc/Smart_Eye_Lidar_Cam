; Auto-generated. Do not edit!


(cl:in-package common-msg)


;//! \htmlinclude ScanMapStatus.msg.html

(cl:defclass <ScanMapStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (scan_status
    :reader scan_status
    :initarg :scan_status
    :type cl:string
    :initform "")
   (file_name
    :reader file_name
    :initarg :file_name
    :type cl:string
    :initform ""))
)

(cl:defclass ScanMapStatus (<ScanMapStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ScanMapStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ScanMapStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name common-msg:<ScanMapStatus> is deprecated: use common-msg:ScanMapStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ScanMapStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common-msg:header-val is deprecated.  Use common-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'scan_status-val :lambda-list '(m))
(cl:defmethod scan_status-val ((m <ScanMapStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common-msg:scan_status-val is deprecated.  Use common-msg:scan_status instead.")
  (scan_status m))

(cl:ensure-generic-function 'file_name-val :lambda-list '(m))
(cl:defmethod file_name-val ((m <ScanMapStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common-msg:file_name-val is deprecated.  Use common-msg:file_name instead.")
  (file_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ScanMapStatus>) ostream)
  "Serializes a message object of type '<ScanMapStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'scan_status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'scan_status))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'file_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'file_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ScanMapStatus>) istream)
  "Deserializes a message object of type '<ScanMapStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'scan_status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'scan_status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'file_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'file_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ScanMapStatus>)))
  "Returns string type for a message object of type '<ScanMapStatus>"
  "common/ScanMapStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ScanMapStatus)))
  "Returns string type for a message object of type 'ScanMapStatus"
  "common/ScanMapStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ScanMapStatus>)))
  "Returns md5sum for a message object of type '<ScanMapStatus>"
  "99a910d59af2cd6d4413707fa4ffd257")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ScanMapStatus)))
  "Returns md5sum for a message object of type 'ScanMapStatus"
  "99a910d59af2cd6d4413707fa4ffd257")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ScanMapStatus>)))
  "Returns full string definition for message of type '<ScanMapStatus>"
  (cl:format cl:nil "# ScanMapStatus.msg~%~%# Standard ROS message header~%#   stamp: time of status update~%#   frame_id: optional, can be empty~%std_msgs/Header header~%~%# Scan status~%# Valid values: idle, scanning, completed, failed~%string scan_status~%~%# File name generated after scan is completed (if any)~%string file_name~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ScanMapStatus)))
  "Returns full string definition for message of type 'ScanMapStatus"
  (cl:format cl:nil "# ScanMapStatus.msg~%~%# Standard ROS message header~%#   stamp: time of status update~%#   frame_id: optional, can be empty~%std_msgs/Header header~%~%# Scan status~%# Valid values: idle, scanning, completed, failed~%string scan_status~%~%# File name generated after scan is completed (if any)~%string file_name~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ScanMapStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'scan_status))
     4 (cl:length (cl:slot-value msg 'file_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ScanMapStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'ScanMapStatus
    (cl:cons ':header (header msg))
    (cl:cons ':scan_status (scan_status msg))
    (cl:cons ':file_name (file_name msg))
))
