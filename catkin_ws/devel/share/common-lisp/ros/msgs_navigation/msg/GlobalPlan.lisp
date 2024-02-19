; Auto-generated. Do not edit!


(cl:in-package msgs_navigation-msg)


;//! \htmlinclude GlobalPlan.msg.html

(cl:defclass <GlobalPlan> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (points
    :reader points
    :initarg :points
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (road_options
    :reader road_options
    :initarg :road_options
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass GlobalPlan (<GlobalPlan>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GlobalPlan>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GlobalPlan)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msgs_navigation-msg:<GlobalPlan> is deprecated: use msgs_navigation-msg:GlobalPlan instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GlobalPlan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_navigation-msg:header-val is deprecated.  Use msgs_navigation-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <GlobalPlan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_navigation-msg:points-val is deprecated.  Use msgs_navigation-msg:points instead.")
  (points m))

(cl:ensure-generic-function 'road_options-val :lambda-list '(m))
(cl:defmethod road_options-val ((m <GlobalPlan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_navigation-msg:road_options-val is deprecated.  Use msgs_navigation-msg:road_options instead.")
  (road_options m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GlobalPlan>)))
    "Constants for message type '<GlobalPlan>"
  '((:LANEFOLLOW . 0)
    (:STRAIGHT . 1)
    (:RIGHT . 2)
    (:LEFT . 3)
    (:CHANGELANELEFT . 4)
    (:CHANGELANERIGHT . 5)
    (:UNKNOWN . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GlobalPlan)))
    "Constants for message type 'GlobalPlan"
  '((:LANEFOLLOW . 0)
    (:STRAIGHT . 1)
    (:RIGHT . 2)
    (:LEFT . 3)
    (:CHANGELANELEFT . 4)
    (:CHANGELANERIGHT . 5)
    (:UNKNOWN . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GlobalPlan>) ostream)
  "Serializes a message object of type '<GlobalPlan>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'road_options))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'road_options))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GlobalPlan>) istream)
  "Deserializes a message object of type '<GlobalPlan>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'road_options) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'road_options)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GlobalPlan>)))
  "Returns string type for a message object of type '<GlobalPlan>"
  "msgs_navigation/GlobalPlan")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GlobalPlan)))
  "Returns string type for a message object of type 'GlobalPlan"
  "msgs_navigation/GlobalPlan")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GlobalPlan>)))
  "Returns md5sum for a message object of type '<GlobalPlan>"
  "30bdb54aae74b679c913a2e5f20b612e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GlobalPlan)))
  "Returns md5sum for a message object of type 'GlobalPlan"
  "30bdb54aae74b679c913a2e5f20b612e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GlobalPlan>)))
  "Returns full string definition for message of type '<GlobalPlan>"
  (cl:format cl:nil "std_msgs/Header header~%~%geometry_msgs/Point[] points~%int32[] road_options~%~%uint8 LANEFOLLOW=0~%uint8 STRAIGHT=1~%uint8 RIGHT=2~%uint8 LEFT=3~%uint8 CHANGELANELEFT=4~%uint8 CHANGELANERIGHT=5~%uint8 UNKNOWN=6~%~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GlobalPlan)))
  "Returns full string definition for message of type 'GlobalPlan"
  (cl:format cl:nil "std_msgs/Header header~%~%geometry_msgs/Point[] points~%int32[] road_options~%~%uint8 LANEFOLLOW=0~%uint8 STRAIGHT=1~%uint8 RIGHT=2~%uint8 LEFT=3~%uint8 CHANGELANELEFT=4~%uint8 CHANGELANERIGHT=5~%uint8 UNKNOWN=6~%~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GlobalPlan>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'road_options) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GlobalPlan>))
  "Converts a ROS message object to a list"
  (cl:list 'GlobalPlan
    (cl:cons ':header (header msg))
    (cl:cons ':points (points msg))
    (cl:cons ':road_options (road_options msg))
))
