; Auto-generated. Do not edit!


(cl:in-package msgs_action-msg)


;//! \htmlinclude SteeringAngle.msg.html

(cl:defclass <SteeringAngle> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass SteeringAngle (<SteeringAngle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SteeringAngle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SteeringAngle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msgs_action-msg:<SteeringAngle> is deprecated: use msgs_action-msg:SteeringAngle instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SteeringAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_action-msg:header-val is deprecated.  Use msgs_action-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <SteeringAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_action-msg:angle-val is deprecated.  Use msgs_action-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SteeringAngle>) ostream)
  "Serializes a message object of type '<SteeringAngle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SteeringAngle>) istream)
  "Deserializes a message object of type '<SteeringAngle>"
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
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SteeringAngle>)))
  "Returns string type for a message object of type '<SteeringAngle>"
  "msgs_action/SteeringAngle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SteeringAngle)))
  "Returns string type for a message object of type 'SteeringAngle"
  "msgs_action/SteeringAngle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SteeringAngle>)))
  "Returns md5sum for a message object of type '<SteeringAngle>"
  "84c1d14f72a90efbf3b1a4de632c5bfb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SteeringAngle)))
  "Returns md5sum for a message object of type 'SteeringAngle"
  "84c1d14f72a90efbf3b1a4de632c5bfb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SteeringAngle>)))
  "Returns full string definition for message of type '<SteeringAngle>"
  (cl:format cl:nil "# This represents the angle of steering system (in radians).  ~%~%Header header~%float64 angle~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SteeringAngle)))
  "Returns full string definition for message of type 'SteeringAngle"
  (cl:format cl:nil "# This represents the angle of steering system (in radians).  ~%~%Header header~%float64 angle~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SteeringAngle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SteeringAngle>))
  "Converts a ROS message object to a list"
  (cl:list 'SteeringAngle
    (cl:cons ':header (header msg))
    (cl:cons ':angle (angle msg))
))
