; Auto-generated. Do not edit!


(cl:in-package msgs_navigation-msg)


;//! \htmlinclude EmergencyStop.msg.html

(cl:defclass <EmergencyStop> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (dist_to_stop
    :reader dist_to_stop
    :initarg :dist_to_stop
    :type cl:float
    :initform 0.0)
   (obstacle
    :reader obstacle
    :initarg :obstacle
    :type msgs_perception-msg:Obstacle
    :initform (cl:make-instance 'msgs_perception-msg:Obstacle))
   (stop_now
    :reader stop_now
    :initarg :stop_now
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass EmergencyStop (<EmergencyStop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EmergencyStop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EmergencyStop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msgs_navigation-msg:<EmergencyStop> is deprecated: use msgs_navigation-msg:EmergencyStop instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EmergencyStop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_navigation-msg:header-val is deprecated.  Use msgs_navigation-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'dist_to_stop-val :lambda-list '(m))
(cl:defmethod dist_to_stop-val ((m <EmergencyStop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_navigation-msg:dist_to_stop-val is deprecated.  Use msgs_navigation-msg:dist_to_stop instead.")
  (dist_to_stop m))

(cl:ensure-generic-function 'obstacle-val :lambda-list '(m))
(cl:defmethod obstacle-val ((m <EmergencyStop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_navigation-msg:obstacle-val is deprecated.  Use msgs_navigation-msg:obstacle instead.")
  (obstacle m))

(cl:ensure-generic-function 'stop_now-val :lambda-list '(m))
(cl:defmethod stop_now-val ((m <EmergencyStop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_navigation-msg:stop_now-val is deprecated.  Use msgs_navigation-msg:stop_now instead.")
  (stop_now m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EmergencyStop>) ostream)
  "Serializes a message object of type '<EmergencyStop>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'dist_to_stop))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obstacle) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stop_now) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EmergencyStop>) istream)
  "Deserializes a message object of type '<EmergencyStop>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dist_to_stop) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obstacle) istream)
    (cl:setf (cl:slot-value msg 'stop_now) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EmergencyStop>)))
  "Returns string type for a message object of type '<EmergencyStop>"
  "msgs_navigation/EmergencyStop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EmergencyStop)))
  "Returns string type for a message object of type 'EmergencyStop"
  "msgs_navigation/EmergencyStop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EmergencyStop>)))
  "Returns md5sum for a message object of type '<EmergencyStop>"
  "6c63fac0d11cdcd3fb7f902e34941777")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EmergencyStop)))
  "Returns md5sum for a message object of type 'EmergencyStop"
  "6c63fac0d11cdcd3fb7f902e34941777")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EmergencyStop>)))
  "Returns full string definition for message of type '<EmergencyStop>"
  (cl:format cl:nil "std_msgs/Header header~%float64 dist_to_stop~%msgs_perception/Obstacle obstacle~%bool stop_now #true to stop at the moment the behavior received the message~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: msgs_perception/Obstacle~%# header for time/frame information~%Header header~%~%#object position ~%int32   id                                # object ID useful in conjunction with the namespace for manipulating and deleting the object later~%string  ns                    		  # Namespace to place this object in... used in conjunction with id to create a unique name for the object~%geometry_msgs/Pose    pose    		  # object pose and orientation~%geometry_msgs/Twist   twist  		  # object velocity~%geometry_msgs/Vector3 linear_acceleration # object acceleration~%geometry_msgs/Vector3 scale       	  # Scale of the object 1,1,1 means (usually 1 meter square)~%~%#Fusion extras~%sensor_msgs/PointCloud2 point_cloud~%int32 class_id~%string[] classes~%msgs_perception/BoundingBox bbox~%~%#Radar extras~%bool    oncoming~%float64 lat_rate~%int32   track_status         # 0 = no target, 1 = new target, 2 = new updated target, 3 = updated target, 4 = coasted target, 5 = merged target, 6 = invalid coasted target, 7 = new coasted target~%bool    bridge_object        # connects two or more objects that are associated with the same obstacle~%~%#visualization extras~%std_msgs/ColorRGBA color          # Color [0.0-1.0]~%duration lifetime                 # How long the object should last before being automatically deleted.  0 means forever~%bool     frame_locked             # If this marker should be frame-locked, i.e. retransformed into its frame every timestep~%float32  animation_speed          # Speed of animation,~%int32    action 	          # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects~%uint8    ADD=0~%uint8    MODIFY=0~%uint8    DELETE=2~%int32 type                 		      # object type : -1 unknown~%string vehicle_model~%~%# based on Marker, See http://www.ros.org/wiki/rviz/DisplayTypes/Marker~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: msgs_perception/BoundingBox~%std_msgs/String classe~%~%geometry_msgs/Point p1~%geometry_msgs/Point p2~%geometry_msgs/Point p3~%geometry_msgs/Point p4~%~%float64 probability~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EmergencyStop)))
  "Returns full string definition for message of type 'EmergencyStop"
  (cl:format cl:nil "std_msgs/Header header~%float64 dist_to_stop~%msgs_perception/Obstacle obstacle~%bool stop_now #true to stop at the moment the behavior received the message~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: msgs_perception/Obstacle~%# header for time/frame information~%Header header~%~%#object position ~%int32   id                                # object ID useful in conjunction with the namespace for manipulating and deleting the object later~%string  ns                    		  # Namespace to place this object in... used in conjunction with id to create a unique name for the object~%geometry_msgs/Pose    pose    		  # object pose and orientation~%geometry_msgs/Twist   twist  		  # object velocity~%geometry_msgs/Vector3 linear_acceleration # object acceleration~%geometry_msgs/Vector3 scale       	  # Scale of the object 1,1,1 means (usually 1 meter square)~%~%#Fusion extras~%sensor_msgs/PointCloud2 point_cloud~%int32 class_id~%string[] classes~%msgs_perception/BoundingBox bbox~%~%#Radar extras~%bool    oncoming~%float64 lat_rate~%int32   track_status         # 0 = no target, 1 = new target, 2 = new updated target, 3 = updated target, 4 = coasted target, 5 = merged target, 6 = invalid coasted target, 7 = new coasted target~%bool    bridge_object        # connects two or more objects that are associated with the same obstacle~%~%#visualization extras~%std_msgs/ColorRGBA color          # Color [0.0-1.0]~%duration lifetime                 # How long the object should last before being automatically deleted.  0 means forever~%bool     frame_locked             # If this marker should be frame-locked, i.e. retransformed into its frame every timestep~%float32  animation_speed          # Speed of animation,~%int32    action 	          # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects~%uint8    ADD=0~%uint8    MODIFY=0~%uint8    DELETE=2~%int32 type                 		      # object type : -1 unknown~%string vehicle_model~%~%# based on Marker, See http://www.ros.org/wiki/rviz/DisplayTypes/Marker~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: msgs_perception/BoundingBox~%std_msgs/String classe~%~%geometry_msgs/Point p1~%geometry_msgs/Point p2~%geometry_msgs/Point p3~%geometry_msgs/Point p4~%~%float64 probability~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EmergencyStop>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obstacle))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EmergencyStop>))
  "Converts a ROS message object to a list"
  (cl:list 'EmergencyStop
    (cl:cons ':header (header msg))
    (cl:cons ':dist_to_stop (dist_to_stop msg))
    (cl:cons ':obstacle (obstacle msg))
    (cl:cons ':stop_now (stop_now msg))
))
