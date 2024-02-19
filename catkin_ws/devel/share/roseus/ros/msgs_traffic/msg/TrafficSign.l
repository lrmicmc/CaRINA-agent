;; Auto-generated. Do not edit!


(when (boundp 'msgs_traffic::TrafficSign)
  (if (not (find-package "MSGS_TRAFFIC"))
    (make-package "MSGS_TRAFFIC"))
  (shadow 'TrafficSign (find-package "MSGS_TRAFFIC")))
(unless (find-package "MSGS_TRAFFIC::TRAFFICSIGN")
  (make-package "MSGS_TRAFFIC::TRAFFICSIGN"))

(in-package "ROS")
;;//! \htmlinclude TrafficSign.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass msgs_traffic::TrafficSign
  :super ros::object
  :slots (_type _name _list _value _traffic_light _pose _length _road_id _lanes ))

(defmethod msgs_traffic::TrafficSign
  (:init
   (&key
    ((:type __type) 0)
    ((:name __name) "")
    ((:list __list) (instance msgs_traffic::signs :init))
    ((:value __value) 0.0)
    ((:traffic_light __traffic_light) (instance msgs_traffic::traffic_light :init))
    ((:pose __pose) (instance geometry_msgs::PoseStamped :init))
    ((:length __length) 0.0)
    ((:road_id __road_id) 0)
    ((:lanes __lanes) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _name (string __name))
   (setq _list __list)
   (setq _value (float __value))
   (setq _traffic_light __traffic_light)
   (setq _pose __pose)
   (setq _length (float __length))
   (setq _road_id (round __road_id))
   (setq _lanes __lanes)
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:list
   (&rest __list)
   (if (keywordp (car __list))
       (send* _list __list)
     (progn
       (if __list (setq _list (car __list)))
       _list)))
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:traffic_light
   (&rest __traffic_light)
   (if (keywordp (car __traffic_light))
       (send* _traffic_light __traffic_light)
     (progn
       (if __traffic_light (setq _traffic_light (car __traffic_light)))
       _traffic_light)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:road_id
   (&optional __road_id)
   (if __road_id (setq _road_id __road_id)) _road_id)
  (:lanes
   (&optional __lanes)
   (if __lanes (setq _lanes __lanes)) _lanes)
  (:serialization-length
   ()
   (+
    ;; uint8 _type
    1
    ;; string _name
    4 (length _name)
    ;; msgs_traffic/signs _list
    (send _list :serialization-length)
    ;; float64 _value
    8
    ;; msgs_traffic/traffic_light _traffic_light
    (send _traffic_light :serialization-length)
    ;; geometry_msgs/PoseStamped _pose
    (send _pose :serialization-length)
    ;; float64 _length
    8
    ;; uint32 _road_id
    4
    ;; uint32[] _lanes
    (* 4    (length _lanes)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _type
       (write-byte _type s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; msgs_traffic/signs _list
       (send _list :serialize s)
     ;; float64 _value
       (sys::poke _value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; msgs_traffic/traffic_light _traffic_light
       (send _traffic_light :serialize s)
     ;; geometry_msgs/PoseStamped _pose
       (send _pose :serialize s)
     ;; float64 _length
       (sys::poke _length (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint32 _road_id
       (write-long _road_id s)
     ;; uint32[] _lanes
     (write-long (length _lanes) s)
     (dotimes (i (length _lanes))
       (write-long (elt _lanes i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; msgs_traffic/signs _list
     (send _list :deserialize buf ptr-) (incf ptr- (send _list :serialization-length))
   ;; float64 _value
     (setq _value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; msgs_traffic/traffic_light _traffic_light
     (send _traffic_light :deserialize buf ptr-) (incf ptr- (send _traffic_light :serialization-length))
   ;; geometry_msgs/PoseStamped _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; float64 _length
     (setq _length (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint32 _road_id
     (setq _road_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32[] _lanes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lanes (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _lanes i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get msgs_traffic::TrafficSign :md5sum-) "a4364772549aa27d16150f86259ed928")
(setf (get msgs_traffic::TrafficSign :datatype-) "msgs_traffic/TrafficSign")
(setf (get msgs_traffic::TrafficSign :definition-)
      "#traffic sign classification
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



(provide :msgs_traffic/TrafficSign "a4364772549aa27d16150f86259ed928")


