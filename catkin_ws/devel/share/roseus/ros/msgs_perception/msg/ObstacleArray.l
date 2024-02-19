;; Auto-generated. Do not edit!


(when (boundp 'msgs_perception::ObstacleArray)
  (if (not (find-package "MSGS_PERCEPTION"))
    (make-package "MSGS_PERCEPTION"))
  (shadow 'ObstacleArray (find-package "MSGS_PERCEPTION")))
(unless (find-package "MSGS_PERCEPTION::OBSTACLEARRAY")
  (make-package "MSGS_PERCEPTION::OBSTACLEARRAY"))

(in-package "ROS")
;;//! \htmlinclude ObstacleArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass msgs_perception::ObstacleArray
  :super ros::object
  :slots (_header _obstacle ))

(defmethod msgs_perception::ObstacleArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:obstacle __obstacle) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _obstacle __obstacle)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:obstacle
   (&rest __obstacle)
   (if (keywordp (car __obstacle))
       (send* _obstacle __obstacle)
     (progn
       (if __obstacle (setq _obstacle (car __obstacle)))
       _obstacle)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; msgs_perception/Obstacle[] _obstacle
    (apply #'+ (send-all _obstacle :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; msgs_perception/Obstacle[] _obstacle
     (write-long (length _obstacle) s)
     (dolist (elem _obstacle)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; msgs_perception/Obstacle[] _obstacle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacle (let (r) (dotimes (i n) (push (instance msgs_perception::Obstacle :init) r)) r))
     (dolist (elem- _obstacle)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get msgs_perception::ObstacleArray :md5sum-) "f42dc2f490652df3c5d675ac2d304dc0")
(setf (get msgs_perception::ObstacleArray :datatype-) "msgs_perception/ObstacleArray")
(setf (get msgs_perception::ObstacleArray :definition-)
      "std_msgs/Header header

msgs_perception/Obstacle[] obstacle

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
MSG: msgs_perception/Obstacle
# header for time/frame information
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



(provide :msgs_perception/ObstacleArray "f42dc2f490652df3c5d675ac2d304dc0")


