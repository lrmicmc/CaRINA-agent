;; Auto-generated. Do not edit!


(when (boundp 'msgs_perception::Obstacle)
  (if (not (find-package "MSGS_PERCEPTION"))
    (make-package "MSGS_PERCEPTION"))
  (shadow 'Obstacle (find-package "MSGS_PERCEPTION")))
(unless (find-package "MSGS_PERCEPTION::OBSTACLE")
  (make-package "MSGS_PERCEPTION::OBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude Obstacle.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*ADD*" (find-package "MSGS_PERCEPTION::OBSTACLE"))
(shadow '*ADD* (find-package "MSGS_PERCEPTION::OBSTACLE"))
(defconstant msgs_perception::Obstacle::*ADD* 0)
(intern "*MODIFY*" (find-package "MSGS_PERCEPTION::OBSTACLE"))
(shadow '*MODIFY* (find-package "MSGS_PERCEPTION::OBSTACLE"))
(defconstant msgs_perception::Obstacle::*MODIFY* 0)
(intern "*DELETE*" (find-package "MSGS_PERCEPTION::OBSTACLE"))
(shadow '*DELETE* (find-package "MSGS_PERCEPTION::OBSTACLE"))
(defconstant msgs_perception::Obstacle::*DELETE* 2)

(defun msgs_perception::Obstacle-to-symbol (const)
  (cond
        ((= const 0) 'msgs_perception::Obstacle::*ADD*)
        ((= const 0) 'msgs_perception::Obstacle::*MODIFY*)
        ((= const 2) 'msgs_perception::Obstacle::*DELETE*)
        (t nil)))

(defclass msgs_perception::Obstacle
  :super ros::object
  :slots (_header _id _ns _pose _twist _linear_acceleration _scale _point_cloud _class_id _classes _bbox _oncoming _lat_rate _track_status _bridge_object _color _lifetime _frame_locked _animation_speed _action _type _vehicle_model ))

(defmethod msgs_perception::Obstacle
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:ns __ns) "")
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:twist __twist) (instance geometry_msgs::Twist :init))
    ((:linear_acceleration __linear_acceleration) (instance geometry_msgs::Vector3 :init))
    ((:scale __scale) (instance geometry_msgs::Vector3 :init))
    ((:point_cloud __point_cloud) (instance sensor_msgs::PointCloud2 :init))
    ((:class_id __class_id) 0)
    ((:classes __classes) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:bbox __bbox) (instance msgs_perception::BoundingBox :init))
    ((:oncoming __oncoming) nil)
    ((:lat_rate __lat_rate) 0.0)
    ((:track_status __track_status) 0)
    ((:bridge_object __bridge_object) nil)
    ((:color __color) (instance std_msgs::ColorRGBA :init))
    ((:lifetime __lifetime) (instance ros::time :init))
    ((:frame_locked __frame_locked) nil)
    ((:animation_speed __animation_speed) 0.0)
    ((:action __action) 0)
    ((:type __type) 0)
    ((:vehicle_model __vehicle_model) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _id (round __id))
   (setq _ns (string __ns))
   (setq _pose __pose)
   (setq _twist __twist)
   (setq _linear_acceleration __linear_acceleration)
   (setq _scale __scale)
   (setq _point_cloud __point_cloud)
   (setq _class_id (round __class_id))
   (setq _classes __classes)
   (setq _bbox __bbox)
   (setq _oncoming __oncoming)
   (setq _lat_rate (float __lat_rate))
   (setq _track_status (round __track_status))
   (setq _bridge_object __bridge_object)
   (setq _color __color)
   (setq _lifetime __lifetime)
   (setq _frame_locked __frame_locked)
   (setq _animation_speed (float __animation_speed))
   (setq _action (round __action))
   (setq _type (round __type))
   (setq _vehicle_model (string __vehicle_model))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:ns
   (&optional __ns)
   (if __ns (setq _ns __ns)) _ns)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
  (:linear_acceleration
   (&rest __linear_acceleration)
   (if (keywordp (car __linear_acceleration))
       (send* _linear_acceleration __linear_acceleration)
     (progn
       (if __linear_acceleration (setq _linear_acceleration (car __linear_acceleration)))
       _linear_acceleration)))
  (:scale
   (&rest __scale)
   (if (keywordp (car __scale))
       (send* _scale __scale)
     (progn
       (if __scale (setq _scale (car __scale)))
       _scale)))
  (:point_cloud
   (&rest __point_cloud)
   (if (keywordp (car __point_cloud))
       (send* _point_cloud __point_cloud)
     (progn
       (if __point_cloud (setq _point_cloud (car __point_cloud)))
       _point_cloud)))
  (:class_id
   (&optional __class_id)
   (if __class_id (setq _class_id __class_id)) _class_id)
  (:classes
   (&optional __classes)
   (if __classes (setq _classes __classes)) _classes)
  (:bbox
   (&rest __bbox)
   (if (keywordp (car __bbox))
       (send* _bbox __bbox)
     (progn
       (if __bbox (setq _bbox (car __bbox)))
       _bbox)))
  (:oncoming
   (&optional (__oncoming :null))
   (if (not (eq __oncoming :null)) (setq _oncoming __oncoming)) _oncoming)
  (:lat_rate
   (&optional __lat_rate)
   (if __lat_rate (setq _lat_rate __lat_rate)) _lat_rate)
  (:track_status
   (&optional __track_status)
   (if __track_status (setq _track_status __track_status)) _track_status)
  (:bridge_object
   (&optional (__bridge_object :null))
   (if (not (eq __bridge_object :null)) (setq _bridge_object __bridge_object)) _bridge_object)
  (:color
   (&rest __color)
   (if (keywordp (car __color))
       (send* _color __color)
     (progn
       (if __color (setq _color (car __color)))
       _color)))
  (:lifetime
   (&optional __lifetime)
   (if __lifetime (setq _lifetime __lifetime)) _lifetime)
  (:frame_locked
   (&optional (__frame_locked :null))
   (if (not (eq __frame_locked :null)) (setq _frame_locked __frame_locked)) _frame_locked)
  (:animation_speed
   (&optional __animation_speed)
   (if __animation_speed (setq _animation_speed __animation_speed)) _animation_speed)
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:vehicle_model
   (&optional __vehicle_model)
   (if __vehicle_model (setq _vehicle_model __vehicle_model)) _vehicle_model)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _id
    4
    ;; string _ns
    4 (length _ns)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Twist _twist
    (send _twist :serialization-length)
    ;; geometry_msgs/Vector3 _linear_acceleration
    (send _linear_acceleration :serialization-length)
    ;; geometry_msgs/Vector3 _scale
    (send _scale :serialization-length)
    ;; sensor_msgs/PointCloud2 _point_cloud
    (send _point_cloud :serialization-length)
    ;; int32 _class_id
    4
    ;; string[] _classes
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _classes)) 4
    ;; msgs_perception/BoundingBox _bbox
    (send _bbox :serialization-length)
    ;; bool _oncoming
    1
    ;; float64 _lat_rate
    8
    ;; int32 _track_status
    4
    ;; bool _bridge_object
    1
    ;; std_msgs/ColorRGBA _color
    (send _color :serialization-length)
    ;; duration _lifetime
    8
    ;; bool _frame_locked
    1
    ;; float32 _animation_speed
    4
    ;; int32 _action
    4
    ;; int32 _type
    4
    ;; string _vehicle_model
    4 (length _vehicle_model)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _id
       (write-long _id s)
     ;; string _ns
       (write-long (length _ns) s) (princ _ns s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Twist _twist
       (send _twist :serialize s)
     ;; geometry_msgs/Vector3 _linear_acceleration
       (send _linear_acceleration :serialize s)
     ;; geometry_msgs/Vector3 _scale
       (send _scale :serialize s)
     ;; sensor_msgs/PointCloud2 _point_cloud
       (send _point_cloud :serialize s)
     ;; int32 _class_id
       (write-long _class_id s)
     ;; string[] _classes
     (write-long (length _classes) s)
     (dolist (elem _classes)
       (write-long (length elem) s) (princ elem s)
       )
     ;; msgs_perception/BoundingBox _bbox
       (send _bbox :serialize s)
     ;; bool _oncoming
       (if _oncoming (write-byte -1 s) (write-byte 0 s))
     ;; float64 _lat_rate
       (sys::poke _lat_rate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _track_status
       (write-long _track_status s)
     ;; bool _bridge_object
       (if _bridge_object (write-byte -1 s) (write-byte 0 s))
     ;; std_msgs/ColorRGBA _color
       (send _color :serialize s)
     ;; duration _lifetime
       (write-long (send _lifetime :sec) s) (write-long (send _lifetime :nsec) s)
     ;; bool _frame_locked
       (if _frame_locked (write-byte -1 s) (write-byte 0 s))
     ;; float32 _animation_speed
       (sys::poke _animation_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _action
       (write-long _action s)
     ;; int32 _type
       (write-long _type s)
     ;; string _vehicle_model
       (write-long (length _vehicle_model) s) (princ _vehicle_model s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _ns
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ns (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Twist _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;; geometry_msgs/Vector3 _linear_acceleration
     (send _linear_acceleration :deserialize buf ptr-) (incf ptr- (send _linear_acceleration :serialization-length))
   ;; geometry_msgs/Vector3 _scale
     (send _scale :deserialize buf ptr-) (incf ptr- (send _scale :serialization-length))
   ;; sensor_msgs/PointCloud2 _point_cloud
     (send _point_cloud :deserialize buf ptr-) (incf ptr- (send _point_cloud :serialization-length))
   ;; int32 _class_id
     (setq _class_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string[] _classes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _classes (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _classes i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; msgs_perception/BoundingBox _bbox
     (send _bbox :deserialize buf ptr-) (incf ptr- (send _bbox :serialization-length))
   ;; bool _oncoming
     (setq _oncoming (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _lat_rate
     (setq _lat_rate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _track_status
     (setq _track_status (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _bridge_object
     (setq _bridge_object (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; std_msgs/ColorRGBA _color
     (send _color :deserialize buf ptr-) (incf ptr- (send _color :serialization-length))
   ;; duration _lifetime
     (send _lifetime :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _lifetime :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _frame_locked
     (setq _frame_locked (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _animation_speed
     (setq _animation_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _action
     (setq _action (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _vehicle_model
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _vehicle_model (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get msgs_perception::Obstacle :md5sum-) "9dd38fb85c319c66ba1631ab9f3d5a09")
(setf (get msgs_perception::Obstacle :datatype-) "msgs_perception/Obstacle")
(setf (get msgs_perception::Obstacle :definition-)
      "# header for time/frame information
Header header

#object position 
int32   id                                # object ID useful in conjunction with the namespace for manipulating and deleting the object later
string  ns                    		  # Namespace to place this object in... used in conjunction with id to create a unique name for the object
geometry_msgs/Pose    pose    		  # object pose and orientation
geometry_msgs/Twist   twist  		  # object velocity
geometry_msgs/Vector3 linear_acceleration # object acceleration
geometry_msgs/Vector3 scale       	  # Scale of the object 1,1,1 means (usually 1 meter square)

#Fusion extras
sensor_msgs/PointCloud2 point_cloud
int32 class_id
string[] classes
msgs_perception/BoundingBox bbox

#Radar extras
bool    oncoming
float64 lat_rate
int32   track_status         # 0 = no target, 1 = new target, 2 = new updated target, 3 = updated target, 4 = coasted target, 5 = merged target, 6 = invalid coasted target, 7 = new coasted target
bool    bridge_object        # connects two or more objects that are associated with the same obstacle

#visualization extras
std_msgs/ColorRGBA color          # Color [0.0-1.0]
duration lifetime                 # How long the object should last before being automatically deleted.  0 means forever
bool     frame_locked             # If this marker should be frame-locked, i.e. retransformed into its frame every timestep
float32  animation_speed          # Speed of animation,
int32    action 	          # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
uint8    ADD=0
uint8    MODIFY=0
uint8    DELETE=2
int32 type                 		      # object type : -1 unknown
string vehicle_model

# based on Marker, See http://www.ros.org/wiki/rviz/DisplayTypes/Marker

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

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

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
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a

")



(provide :msgs_perception/Obstacle "9dd38fb85c319c66ba1631ab9f3d5a09")


