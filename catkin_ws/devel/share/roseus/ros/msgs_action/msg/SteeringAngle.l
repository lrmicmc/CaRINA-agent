;; Auto-generated. Do not edit!


(when (boundp 'msgs_action::SteeringAngle)
  (if (not (find-package "MSGS_ACTION"))
    (make-package "MSGS_ACTION"))
  (shadow 'SteeringAngle (find-package "MSGS_ACTION")))
(unless (find-package "MSGS_ACTION::STEERINGANGLE")
  (make-package "MSGS_ACTION::STEERINGANGLE"))

(in-package "ROS")
;;//! \htmlinclude SteeringAngle.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass msgs_action::SteeringAngle
  :super ros::object
  :slots (_header _angle ))

(defmethod msgs_action::SteeringAngle
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _angle (float __angle))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get msgs_action::SteeringAngle :md5sum-) "84c1d14f72a90efbf3b1a4de632c5bfb")
(setf (get msgs_action::SteeringAngle :datatype-) "msgs_action/SteeringAngle")
(setf (get msgs_action::SteeringAngle :definition-)
      "# This represents the angle of steering system (in radians).  

Header header
float64 angle

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



(provide :msgs_action/SteeringAngle "84c1d14f72a90efbf3b1a4de632c5bfb")


