;; Auto-generated. Do not edit!


(when (boundp 'msgs_mapping::HDMap)
  (if (not (find-package "MSGS_MAPPING"))
    (make-package "MSGS_MAPPING"))
  (shadow 'HDMap (find-package "MSGS_MAPPING")))
(unless (find-package "MSGS_MAPPING::HDMAP")
  (make-package "MSGS_MAPPING::HDMAP"))

(in-package "ROS")
;;//! \htmlinclude HDMap.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass msgs_mapping::HDMap
  :super ros::object
  :slots (_header _XML_HDMap _pose ))

(defmethod msgs_mapping::HDMap
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:XML_HDMap __XML_HDMap) "")
    ((:pose __pose) (instance geometry_msgs::PoseWithCovarianceStamped :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _XML_HDMap (string __XML_HDMap))
   (setq _pose __pose)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:XML_HDMap
   (&optional __XML_HDMap)
   (if __XML_HDMap (setq _XML_HDMap __XML_HDMap)) _XML_HDMap)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _XML_HDMap
    4 (length _XML_HDMap)
    ;; geometry_msgs/PoseWithCovarianceStamped _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _XML_HDMap
       (write-long (length _XML_HDMap) s) (princ _XML_HDMap s)
     ;; geometry_msgs/PoseWithCovarianceStamped _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _XML_HDMap
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _XML_HDMap (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/PoseWithCovarianceStamped _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get msgs_mapping::HDMap :md5sum-) "f28f659f2e5da98f786a894149a1b462")
(setf (get msgs_mapping::HDMap :datatype-) "msgs_mapping/HDMap")
(setf (get msgs_mapping::HDMap :definition-)
      "std_msgs/Header header
string XML_HDMap
geometry_msgs/PoseWithCovarianceStamped pose

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

================================================================================
MSG: geometry_msgs/PoseWithCovarianceStamped
# This expresses an estimated pose with a reference coordinate frame and timestamp

Header header
PoseWithCovariance pose

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :msgs_mapping/HDMap "f28f659f2e5da98f786a894149a1b462")


