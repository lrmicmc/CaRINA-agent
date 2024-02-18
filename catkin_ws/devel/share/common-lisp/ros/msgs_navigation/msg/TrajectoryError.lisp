; Auto-generated. Do not edit!


(cl:in-package msgs_navigation-msg)


;//! \htmlinclude TrajectoryError.msg.html

(cl:defclass <TrajectoryError> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (enable
    :reader enable
    :initarg :enable
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 4 :element-type 'cl:boolean :initial-element cl:nil))
   (lateral_error
    :reader lateral_error
    :initarg :lateral_error
    :type cl:float
    :initform 0.0)
   (angular_error
    :reader angular_error
    :initarg :angular_error
    :type cl:float
    :initform 0.0)
   (longitudinal_error
    :reader longitudinal_error
    :initarg :longitudinal_error
    :type cl:float
    :initform 0.0)
   (kappa_error
    :reader kappa_error
    :initarg :kappa_error
    :type cl:float
    :initform 0.0))
)

(cl:defclass TrajectoryError (<TrajectoryError>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryError>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryError)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msgs_navigation-msg:<TrajectoryError> is deprecated: use msgs_navigation-msg:TrajectoryError instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrajectoryError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_navigation-msg:header-val is deprecated.  Use msgs_navigation-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <TrajectoryError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_navigation-msg:enable-val is deprecated.  Use msgs_navigation-msg:enable instead.")
  (enable m))

(cl:ensure-generic-function 'lateral_error-val :lambda-list '(m))
(cl:defmethod lateral_error-val ((m <TrajectoryError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_navigation-msg:lateral_error-val is deprecated.  Use msgs_navigation-msg:lateral_error instead.")
  (lateral_error m))

(cl:ensure-generic-function 'angular_error-val :lambda-list '(m))
(cl:defmethod angular_error-val ((m <TrajectoryError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_navigation-msg:angular_error-val is deprecated.  Use msgs_navigation-msg:angular_error instead.")
  (angular_error m))

(cl:ensure-generic-function 'longitudinal_error-val :lambda-list '(m))
(cl:defmethod longitudinal_error-val ((m <TrajectoryError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_navigation-msg:longitudinal_error-val is deprecated.  Use msgs_navigation-msg:longitudinal_error instead.")
  (longitudinal_error m))

(cl:ensure-generic-function 'kappa_error-val :lambda-list '(m))
(cl:defmethod kappa_error-val ((m <TrajectoryError>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_navigation-msg:kappa_error-val is deprecated.  Use msgs_navigation-msg:kappa_error instead.")
  (kappa_error m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<TrajectoryError>)))
    "Constants for message type '<TrajectoryError>"
  '((:LATERAL . 0)
    (:ANGULAR . 1)
    (:LONGITUDINAL . 2)
    (:KAPPA . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'TrajectoryError)))
    "Constants for message type 'TrajectoryError"
  '((:LATERAL . 0)
    (:ANGULAR . 1)
    (:LONGITUDINAL . 2)
    (:KAPPA . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryError>) ostream)
  "Serializes a message object of type '<TrajectoryError>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'enable))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lateral_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angular_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'longitudinal_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'kappa_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryError>) istream)
  "Deserializes a message object of type '<TrajectoryError>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'enable) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'enable)))
    (cl:dotimes (i 4)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lateral_error) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_error) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitudinal_error) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kappa_error) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryError>)))
  "Returns string type for a message object of type '<TrajectoryError>"
  "msgs_navigation/TrajectoryError")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryError)))
  "Returns string type for a message object of type 'TrajectoryError"
  "msgs_navigation/TrajectoryError")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryError>)))
  "Returns md5sum for a message object of type '<TrajectoryError>"
  "4965b313ef7d8cf00b86bfaf40b54c20")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryError)))
  "Returns md5sum for a message object of type 'TrajectoryError"
  "4965b313ef7d8cf00b86bfaf40b54c20")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryError>)))
  "Returns full string definition for message of type '<TrajectoryError>"
  (cl:format cl:nil "std_msgs/Header header~%~%bool[4] enable~%uint8 LATERAL = 0~%uint8 ANGULAR = 1~%uint8 LONGITUDINAL = 2~%uint8 KAPPA = 3~%~%float64 lateral_error~%float64 angular_error~%float64 longitudinal_error~%float64 kappa_error~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryError)))
  "Returns full string definition for message of type 'TrajectoryError"
  (cl:format cl:nil "std_msgs/Header header~%~%bool[4] enable~%uint8 LATERAL = 0~%uint8 ANGULAR = 1~%uint8 LONGITUDINAL = 2~%uint8 KAPPA = 3~%~%float64 lateral_error~%float64 angular_error~%float64 longitudinal_error~%float64 kappa_error~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryError>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'enable) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryError>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryError
    (cl:cons ':header (header msg))
    (cl:cons ':enable (enable msg))
    (cl:cons ':lateral_error (lateral_error msg))
    (cl:cons ':angular_error (angular_error msg))
    (cl:cons ':longitudinal_error (longitudinal_error msg))
    (cl:cons ':kappa_error (kappa_error msg))
))
