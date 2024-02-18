;; Auto-generated. Do not edit!


(when (boundp 'msgs_perception::BoundingBoxArray)
  (if (not (find-package "MSGS_PERCEPTION"))
    (make-package "MSGS_PERCEPTION"))
  (shadow 'BoundingBoxArray (find-package "MSGS_PERCEPTION")))
(unless (find-package "MSGS_PERCEPTION::BOUNDINGBOXARRAY")
  (make-package "MSGS_PERCEPTION::BOUNDINGBOXARRAY"))

(in-package "ROS")
;;//! \htmlinclude BoundingBoxArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass msgs_perception::BoundingBoxArray
  :super ros::object
  :slots (_header _objects _size ))

(defmethod msgs_perception::BoundingBoxArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:objects __objects) ())
    ((:size __size) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _objects __objects)
   (setq _size (round __size))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:objects
   (&rest __objects)
   (if (keywordp (car __objects))
       (send* _objects __objects)
     (progn
       (if __objects (setq _objects (car __objects)))
       _objects)))
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; msgs_perception/BoundingBox[] _objects
    (apply #'+ (send-all _objects :serialization-length)) 4
    ;; int32 _size
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; msgs_perception/BoundingBox[] _objects
     (write-long (length _objects) s)
     (dolist (elem _objects)
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
   ;; msgs_perception/BoundingBox[] _objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _objects (let (r) (dotimes (i n) (push (instance msgs_perception::BoundingBox :init) r)) r))
     (dolist (elem- _objects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int32 _size
     (setq _size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get msgs_perception::BoundingBoxArray :md5sum-) "394b720a79915851d11a3beb61322fe8")
(setf (get msgs_perception::BoundingBoxArray :datatype-) "msgs_perception/BoundingBoxArray")
(setf (get msgs_perception::BoundingBoxArray :definition-)
      "std_msgs/Header header

msgs_perception/BoundingBox[] objects 

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
MSG: msgs_perception/BoundingBox
std_msgs/String classe

geometry_msgs/Point p1
geometry_msgs/Point p2
geometry_msgs/Point p3
geometry_msgs/Point p4

float64 probability

================================================================================
MSG: std_msgs/String
string data

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :msgs_perception/BoundingBoxArray "394b720a79915851d11a3beb61322fe8")


