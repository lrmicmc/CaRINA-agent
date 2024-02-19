; Auto-generated. Do not edit!


(cl:in-package msgs_action-msg)


;//! \htmlinclude OperationMode.msg.html

(cl:defclass <OperationMode> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass OperationMode (<OperationMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OperationMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OperationMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msgs_action-msg:<OperationMode> is deprecated: use msgs_action-msg:OperationMode instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <OperationMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_action-msg:value-val is deprecated.  Use msgs_action-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<OperationMode>)))
    "Constants for message type '<OperationMode>"
  '((:OPERATIONAL . 0)
    (:EMERGENCY . 1)
    (:MANUAL_ACCELERATION . 0)
    (:AUTO_ACCELERATION . 2)
    (:MANUAL_STEERING . 0)
    (:AUTO_STEERING . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'OperationMode)))
    "Constants for message type 'OperationMode"
  '((:OPERATIONAL . 0)
    (:EMERGENCY . 1)
    (:MANUAL_ACCELERATION . 0)
    (:AUTO_ACCELERATION . 2)
    (:MANUAL_STEERING . 0)
    (:AUTO_STEERING . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OperationMode>) ostream)
  "Serializes a message object of type '<OperationMode>"
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OperationMode>) istream)
  "Deserializes a message object of type '<OperationMode>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OperationMode>)))
  "Returns string type for a message object of type '<OperationMode>"
  "msgs_action/OperationMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OperationMode)))
  "Returns string type for a message object of type 'OperationMode"
  "msgs_action/OperationMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OperationMode>)))
  "Returns md5sum for a message object of type '<OperationMode>"
  "77d89d0ca02e0b8d1ca091934c6fbbea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OperationMode)))
  "Returns md5sum for a message object of type 'OperationMode"
  "77d89d0ca02e0b8d1ca091934c6fbbea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OperationMode>)))
  "Returns full string definition for message of type '<OperationMode>"
  (cl:format cl:nil "# ART Navigator behaviors (lower numbers have higher priority)~%# $Id: Behavior.msg 996 2011-02-27 16:07:34Z jack.oquin $~%~%#Emergency state~%int32 operational = 0~%int32 emergency = 1~%~%#Acceleration state~%int32 manual_acceleration = 0~%int32 auto_acceleration = 2~%~%#Steering state~%int32 manual_steering = 0~%int32 auto_steering = 4~%~%int32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OperationMode)))
  "Returns full string definition for message of type 'OperationMode"
  (cl:format cl:nil "# ART Navigator behaviors (lower numbers have higher priority)~%# $Id: Behavior.msg 996 2011-02-27 16:07:34Z jack.oquin $~%~%#Emergency state~%int32 operational = 0~%int32 emergency = 1~%~%#Acceleration state~%int32 manual_acceleration = 0~%int32 auto_acceleration = 2~%~%#Steering state~%int32 manual_steering = 0~%int32 auto_steering = 4~%~%int32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OperationMode>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OperationMode>))
  "Converts a ROS message object to a list"
  (cl:list 'OperationMode
    (cl:cons ':value (value msg))
))
