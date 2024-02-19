;; Auto-generated. Do not edit!


(when (boundp 'msgs_navigation::Path)
  (if (not (find-package "MSGS_NAVIGATION"))
    (make-package "MSGS_NAVIGATION"))
  (shadow 'Path (find-package "MSGS_NAVIGATION")))
(unless (find-package "MSGS_NAVIGATION::PATH")
  (make-package "MSGS_NAVIGATION::PATH"))

(in-package "ROS")
;;//! \htmlinclude Path.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass msgs_navigation::Path
  :super ros::object
  :slots (_header _path _size ))

(defmethod msgs_navigation::Path
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:path __path) ())
    ((:size __size) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _path __path)
   (setq _size (round __size))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; msgs_navigation/TrajectoryPoint[] _path
    (apply #'+ (send-all _path :serialization-length)) 4
    ;; int32 _size
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; msgs_navigation/TrajectoryPoint[] _path
     (write-long (length _path) s)
     (dolist (elem _path)
       (send elem :serialize s)
       )
     ;; int32 _size
       (write-long _size s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; msgs_navigation/TrajectoryPoint[] _path
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _path (let (r) (dotimes (i n) (push (instance msgs_navigation::TrajectoryPoint :init) r)) r))
     (dolist (elem- _path)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int32 _size
     (setq _size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get msgs_navigation::Path :md5sum-) "9cb18433fe776145fb64e8e568e28b11")
(setf (get msgs_navigation::Path :datatype-) "msgs_navigation/Path")
(setf (get msgs_navigation::Path :definition-)
      "std_msgs/Header header
msgs_navigation/TrajectoryPoint[] path
int32 size


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
MSG: msgs_navigation/TrajectoryPoint
# Clothoid data

float64[] point

uint8 X = 0
uint8 Y = 1
uint8 KAPPA = 2
uint8 ANGLE = 3
uint8 LENGTH = 4
uint8 SPEED = 5
uint8 KAPPA_DERIVATIVE = 6

float64 point_number
bool end_track

")



(provide :msgs_navigation/Path "9cb18433fe776145fb64e8e568e28b11")


