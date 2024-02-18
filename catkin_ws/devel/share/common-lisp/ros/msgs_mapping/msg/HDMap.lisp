; Auto-generated. Do not edit!


(cl:in-package msgs_mapping-msg)


;//! \htmlinclude HDMap.msg.html

(cl:defclass <HDMap> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (XML_HDMap
    :reader XML_HDMap
    :initarg :XML_HDMap
    :type cl:string
    :initform "")
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:PoseWithCovarianceStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseWithCovarianceStamped)))
)

(cl:defclass HDMap (<HDMap>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HDMap>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HDMap)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msgs_mapping-msg:<HDMap> is deprecated: use msgs_mapping-msg:HDMap instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HDMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_mapping-msg:header-val is deprecated.  Use msgs_mapping-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'XML_HDMap-val :lambda-list '(m))
(cl:defmethod XML_HDMap-val ((m <HDMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_mapping-msg:XML_HDMap-val is deprecated.  Use msgs_mapping-msg:XML_HDMap instead.")
  (XML_HDMap m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <HDMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_mapping-msg:pose-val is deprecated.  Use msgs_mapping-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HDMap>) ostream)
  "Serializes a message object of type '<HDMap>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'XML_HDMap))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'XML_HDMap))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HDMap>) istream)
  "Deserializes a message object of type '<HDMap>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'XML_HDMap) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'XML_HDMap) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HDMap>)))
  "Returns string type for a message object of type '<HDMap>"
  "msgs_mapping/HDMap")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HDMap)))
  "Returns string type for a message object of type 'HDMap"
  "msgs_mapping/HDMap")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HDMap>)))
  "Returns md5sum for a message object of type '<HDMap>"
  "f28f659f2e5da98f786a894149a1b462")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HDMap)))
  "Returns md5sum for a message object of type 'HDMap"
  "f28f659f2e5da98f786a894149a1b462")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HDMap>)))
  "Returns full string definition for message of type '<HDMap>"
  (cl:format cl:nil "std_msgs/Header header~%string XML_HDMap~%geometry_msgs/PoseWithCovarianceStamped pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovarianceStamped~%# This expresses an estimated pose with a reference coordinate frame and timestamp~%~%Header header~%PoseWithCovariance pose~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HDMap)))
  "Returns full string definition for message of type 'HDMap"
  (cl:format cl:nil "std_msgs/Header header~%string XML_HDMap~%geometry_msgs/PoseWithCovarianceStamped pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovarianceStamped~%# This expresses an estimated pose with a reference coordinate frame and timestamp~%~%Header header~%PoseWithCovariance pose~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HDMap>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'XML_HDMap))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HDMap>))
  "Converts a ROS message object to a list"
  (cl:list 'HDMap
    (cl:cons ':header (header msg))
    (cl:cons ':XML_HDMap (XML_HDMap msg))
    (cl:cons ':pose (pose msg))
))
