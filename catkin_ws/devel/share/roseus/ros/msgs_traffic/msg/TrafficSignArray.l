;; Auto-generated. Do not edit!


(when (boundp 'msgs_traffic::TrafficSignArray)
  (if (not (find-package "MSGS_TRAFFIC"))
    (make-package "MSGS_TRAFFIC"))
  (shadow 'TrafficSignArray (find-package "MSGS_TRAFFIC")))
(unless (find-package "MSGS_TRAFFIC::TRAFFICSIGNARRAY")
  (make-package "MSGS_TRAFFIC::TRAFFICSIGNARRAY"))

(in-package "ROS")
;;//! \htmlinclude TrafficSignArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass msgs_traffic::TrafficSignArray
  :super ros::object
  :slots (_header _signs ))

(defmethod msgs_traffic::TrafficSignArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:signs __signs) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _signs __signs)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:signs
   (&rest __signs)
   (if (keywordp (car __signs))
       (send* _signs __signs)
     (progn
       (if __signs (setq _signs (car __signs)))
       _signs)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; msgs_traffic/TrafficSign[] _signs
    (apply #'+ (send-all _signs :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; msgs_traffic/TrafficSign[] _signs
     (write-long (length _signs) s)
     (dolist (elem _signs)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; msgs_traffic/TrafficSign[] _signs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _signs (let (r) (dotimes (i n) (push (instance msgs_traffic::TrafficSign :init) r)) r))
     (dolist (elem- _signs)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get msgs_traffic::TrafficSignArray :md5sum-) "52473d5a9fb0a322cd148dfb905888a7")
(setf (get msgs_traffic::TrafficSignArray :datatype-) "msgs_traffic/TrafficSignArray")
(setf (get msgs_traffic::TrafficSignArray :definition-)
      "std_msgs/Header header
msgs_traffic/TrafficSign[] signs

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
MSG: msgs_traffic/TrafficSign
#traffic sign classification
uint8 type
string name
msgs_traffic/signs list

#traffic sign semantic
float64 value
msgs_traffic/traffic_light traffic_light

#traffic sign localization 
geometry_msgs/PoseStamped pose
float64 length
uint32 road_id 
uint32[] lanes






================================================================================
MSG: msgs_traffic/signs
#list of all traffic signs

uint8 UNKNOWN             = 0
uint8 STOP                = 1
uint8 SPEED_LIMIT         = 2
uint8 PEDESTRIAN_CROSSING = 3
uint8 SPEED_BUMP          = 4
uint8 TRAFFIC_LIGHT       = 5

================================================================================
MSG: msgs_traffic/traffic_light
uint8 RED    = 0
uint8 GREEN  = 1
uint8 YELLOW = 2

uint8 color

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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



(provide :msgs_traffic/TrafficSignArray "52473d5a9fb0a322cd148dfb905888a7")


