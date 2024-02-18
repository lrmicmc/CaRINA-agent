; Auto-generated. Do not edit!


(cl:in-package msgs_traffic-msg)


;//! \htmlinclude TrafficSign.msg.html

(cl:defclass <TrafficSign> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (list
    :reader list
    :initarg :list
    :type msgs_traffic-msg:signs
    :initform (cl:make-instance 'msgs_traffic-msg:signs))
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0)
   (traffic_light
    :reader traffic_light
    :initarg :traffic_light
    :type msgs_traffic-msg:traffic_light
    :initform (cl:make-instance 'msgs_traffic-msg:traffic_light))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (length
    :reader length
    :initarg :length
    :type cl:float
    :initform 0.0)
   (road_id
    :reader road_id
    :initarg :road_id
    :type cl:integer
    :initform 0)
   (lanes
    :reader lanes
    :initarg :lanes
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass TrafficSign (<TrafficSign>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrafficSign>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrafficSign)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msgs_traffic-msg:<TrafficSign> is deprecated: use msgs_traffic-msg:TrafficSign instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <TrafficSign>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_traffic-msg:type-val is deprecated.  Use msgs_traffic-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <TrafficSign>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_traffic-msg:name-val is deprecated.  Use msgs_traffic-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'list-val :lambda-list '(m))
(cl:defmethod list-val ((m <TrafficSign>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_traffic-msg:list-val is deprecated.  Use msgs_traffic-msg:list instead.")
  (list m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <TrafficSign>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_traffic-msg:value-val is deprecated.  Use msgs_traffic-msg:value instead.")
  (value m))

(cl:ensure-generic-function 'traffic_light-val :lambda-list '(m))
(cl:defmethod traffic_light-val ((m <TrafficSign>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_traffic-msg:traffic_light-val is deprecated.  Use msgs_traffic-msg:traffic_light instead.")
  (traffic_light m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <TrafficSign>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_traffic-msg:pose-val is deprecated.  Use msgs_traffic-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <TrafficSign>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_traffic-msg:length-val is deprecated.  Use msgs_traffic-msg:length instead.")
  (length m))

(cl:ensure-generic-function 'road_id-val :lambda-list '(m))
(cl:defmethod road_id-val ((m <TrafficSign>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_traffic-msg:road_id-val is deprecated.  Use msgs_traffic-msg:road_id instead.")
  (road_id m))

(cl:ensure-generic-function 'lanes-val :lambda-list '(m))
(cl:defmethod lanes-val ((m <TrafficSign>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_traffic-msg:lanes-val is deprecated.  Use msgs_traffic-msg:lanes instead.")
  (lanes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrafficSign>) ostream)
  "Serializes a message object of type '<TrafficSign>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'list) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'traffic_light) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'road_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'road_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'road_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'road_id)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'lanes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'lanes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrafficSign>) istream)
  "Deserializes a message object of type '<TrafficSign>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'list) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'traffic_light) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'length) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'road_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'road_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'road_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'road_id)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'lanes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'lanes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrafficSign>)))
  "Returns string type for a message object of type '<TrafficSign>"
  "msgs_traffic/TrafficSign")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrafficSign)))
  "Returns string type for a message object of type 'TrafficSign"
  "msgs_traffic/TrafficSign")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrafficSign>)))
  "Returns md5sum for a message object of type '<TrafficSign>"
  "a4364772549aa27d16150f86259ed928")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrafficSign)))
  "Returns md5sum for a message object of type 'TrafficSign"
  "a4364772549aa27d16150f86259ed928")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrafficSign>)))
  "Returns full string definition for message of type '<TrafficSign>"
  (cl:format cl:nil "#traffic sign classification~%uint8 type~%string name~%msgs_traffic/signs list~%~%#traffic sign semantic~%float64 value~%msgs_traffic/traffic_light traffic_light~%~%#traffic sign localization ~%geometry_msgs/PoseStamped pose~%float64 length~%uint32 road_id ~%uint32[] lanes~%~%~%~%~%~%~%================================================================================~%MSG: msgs_traffic/signs~%#list of all traffic signs~%~%uint8 UNKNOWN             = 0~%uint8 STOP                = 1~%uint8 SPEED_LIMIT         = 2~%uint8 PEDESTRIAN_CROSSING = 3~%uint8 SPEED_BUMP          = 4~%uint8 TRAFFIC_LIGHT       = 5~%~%================================================================================~%MSG: msgs_traffic/traffic_light~%uint8 RED    = 0~%uint8 GREEN  = 1~%uint8 YELLOW = 2~%~%uint8 color~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrafficSign)))
  "Returns full string definition for message of type 'TrafficSign"
  (cl:format cl:nil "#traffic sign classification~%uint8 type~%string name~%msgs_traffic/signs list~%~%#traffic sign semantic~%float64 value~%msgs_traffic/traffic_light traffic_light~%~%#traffic sign localization ~%geometry_msgs/PoseStamped pose~%float64 length~%uint32 road_id ~%uint32[] lanes~%~%~%~%~%~%~%================================================================================~%MSG: msgs_traffic/signs~%#list of all traffic signs~%~%uint8 UNKNOWN             = 0~%uint8 STOP                = 1~%uint8 SPEED_LIMIT         = 2~%uint8 PEDESTRIAN_CROSSING = 3~%uint8 SPEED_BUMP          = 4~%uint8 TRAFFIC_LIGHT       = 5~%~%================================================================================~%MSG: msgs_traffic/traffic_light~%uint8 RED    = 0~%uint8 GREEN  = 1~%uint8 YELLOW = 2~%~%uint8 color~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrafficSign>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'list))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'traffic_light))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     8
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'lanes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrafficSign>))
  "Converts a ROS message object to a list"
  (cl:list 'TrafficSign
    (cl:cons ':type (type msg))
    (cl:cons ':name (name msg))
    (cl:cons ':list (list msg))
    (cl:cons ':value (value msg))
    (cl:cons ':traffic_light (traffic_light msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':length (length msg))
    (cl:cons ':road_id (road_id msg))
    (cl:cons ':lanes (lanes msg))
))
