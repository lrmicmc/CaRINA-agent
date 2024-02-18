; Auto-generated. Do not edit!


(cl:in-package msgs_action-msg)


;//! \htmlinclude Throttle.msg.html

(cl:defclass <Throttle> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass Throttle (<Throttle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Throttle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Throttle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msgs_action-msg:<Throttle> is deprecated: use msgs_action-msg:Throttle instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Throttle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_action-msg:header-val is deprecated.  Use msgs_action-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <Throttle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_action-msg:value-val is deprecated.  Use msgs_action-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Throttle>) ostream)
  "Serializes a message object of type '<Throttle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Throttle>) istream)
  "Deserializes a message object of type '<Throttle>"
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
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Throttle>)))
  "Returns string type for a message object of type '<Throttle>"
  "msgs_action/Throttle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Throttle)))
  "Returns string type for a message object of type 'Throttle"
  "msgs_action/Throttle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Throttle>)))
  "Returns md5sum for a message object of type '<Throttle>"
  "d053817de0764f9ee90dbc89c4cdd751")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Throttle)))
  "Returns md5sum for a message object of type 'Throttle"
  "d053817de0764f9ee90dbc89c4cdd751")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Throttle>)))
  "Returns full string definition for message of type '<Throttle>"
  (cl:format cl:nil "Header header~%float64 value	# This represents how many percent will be applied in the throttle.  ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Throttle)))
  "Returns full string definition for message of type 'Throttle"
  (cl:format cl:nil "Header header~%float64 value	# This represents how many percent will be applied in the throttle.  ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Throttle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Throttle>))
  "Converts a ROS message object to a list"
  (cl:list 'Throttle
    (cl:cons ':header (header msg))
    (cl:cons ':value (value msg))
))
