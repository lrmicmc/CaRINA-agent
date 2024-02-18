; Auto-generated. Do not edit!


(cl:in-package msgs_action-msg)


;//! \htmlinclude VehicleState.msg.html

(cl:defclass <VehicleState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (drive
    :reader drive
    :initarg :drive
    :type ackermann_msgs-msg:AckermannDrive
    :initform (cl:make-instance 'ackermann_msgs-msg:AckermannDrive))
   (engine_speed
    :reader engine_speed
    :initarg :engine_speed
    :type cl:integer
    :initform 0)
   (throttle
    :reader throttle
    :initarg :throttle
    :type cl:float
    :initform 0.0)
   (car_gear
    :reader car_gear
    :initarg :car_gear
    :type cl:integer
    :initform 0)
   (brake
    :reader brake
    :initarg :brake
    :type cl:integer
    :initform 0)
   (handbrake
    :reader handbrake
    :initarg :handbrake
    :type cl:integer
    :initform 0))
)

(cl:defclass VehicleState (<VehicleState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VehicleState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VehicleState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msgs_action-msg:<VehicleState> is deprecated: use msgs_action-msg:VehicleState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <VehicleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_action-msg:header-val is deprecated.  Use msgs_action-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'drive-val :lambda-list '(m))
(cl:defmethod drive-val ((m <VehicleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_action-msg:drive-val is deprecated.  Use msgs_action-msg:drive instead.")
  (drive m))

(cl:ensure-generic-function 'engine_speed-val :lambda-list '(m))
(cl:defmethod engine_speed-val ((m <VehicleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_action-msg:engine_speed-val is deprecated.  Use msgs_action-msg:engine_speed instead.")
  (engine_speed m))

(cl:ensure-generic-function 'throttle-val :lambda-list '(m))
(cl:defmethod throttle-val ((m <VehicleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_action-msg:throttle-val is deprecated.  Use msgs_action-msg:throttle instead.")
  (throttle m))

(cl:ensure-generic-function 'car_gear-val :lambda-list '(m))
(cl:defmethod car_gear-val ((m <VehicleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_action-msg:car_gear-val is deprecated.  Use msgs_action-msg:car_gear instead.")
  (car_gear m))

(cl:ensure-generic-function 'brake-val :lambda-list '(m))
(cl:defmethod brake-val ((m <VehicleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_action-msg:brake-val is deprecated.  Use msgs_action-msg:brake instead.")
  (brake m))

(cl:ensure-generic-function 'handbrake-val :lambda-list '(m))
(cl:defmethod handbrake-val ((m <VehicleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_action-msg:handbrake-val is deprecated.  Use msgs_action-msg:handbrake instead.")
  (handbrake m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VehicleState>) ostream)
  "Serializes a message object of type '<VehicleState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'drive) ostream)
  (cl:let* ((signed (cl:slot-value msg 'engine_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'throttle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'car_gear)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'brake)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'handbrake)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VehicleState>) istream)
  "Deserializes a message object of type '<VehicleState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'drive) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'engine_speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'throttle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'car_gear) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'brake) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'handbrake) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VehicleState>)))
  "Returns string type for a message object of type '<VehicleState>"
  "msgs_action/VehicleState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VehicleState)))
  "Returns string type for a message object of type 'VehicleState"
  "msgs_action/VehicleState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VehicleState>)))
  "Returns md5sum for a message object of type '<VehicleState>"
  "d59fbb8f50f426518dc752cfb6444965")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VehicleState)))
  "Returns md5sum for a message object of type 'VehicleState"
  "d59fbb8f50f426518dc752cfb6444965")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VehicleState>)))
  "Returns full string definition for message of type '<VehicleState>"
  (cl:format cl:nil "Header header~%ackermann_msgs/AckermannDrive drive	# Look for ackermann_msgs at ros wiki~%int32 engine_speed					# Engine revolution-counter [range 0 to 10240 rpm]~%float64 throttle					# Throttle pedal position [range: 0 to 99.96%]~%int32 car_gear 						# Gear selection [range: -1 (reverse), 0 (neutral) to 5 (fifth)]~%int32 brake							# Brake active (1) or inactive (0)~%int32 handbrake						# Handbrake active (1) or inactive (0) ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: ackermann_msgs/AckermannDrive~%## Driving command for a car-like vehicle using Ackermann steering.~%#  $Id$~%~%# Assumes Ackermann front-wheel steering. The left and right front~%# wheels are generally at different angles. To simplify, the commanded~%# angle corresponds to the yaw of a virtual wheel located at the~%# center of the front axle, like on a tricycle.  Positive yaw is to~%# the left. (This is *not* the angle of the steering wheel inside the~%# passenger compartment.)~%#~%# Zero steering angle velocity means change the steering angle as~%# quickly as possible. Positive velocity indicates a desired absolute~%# rate of change either left or right. The controller tries not to~%# exceed this limit in either direction, but sometimes it might.~%#~%float32 steering_angle          # desired virtual angle (radians)~%float32 steering_angle_velocity # desired rate of change (radians/s)~%~%# Drive at requested speed, acceleration and jerk (the 1st, 2nd and~%# 3rd derivatives of position). All are measured at the vehicle's~%# center of rotation, typically the center of the rear axle. The~%# controller tries not to exceed these limits in either direction, but~%# sometimes it might.~%#~%# Speed is the desired scalar magnitude of the velocity vector.~%# Direction is forward unless the sign is negative, indicating reverse.~%#~%# Zero acceleration means change speed as quickly as~%# possible. Positive acceleration indicates a desired absolute~%# magnitude; that includes deceleration.~%#~%# Zero jerk means change acceleration as quickly as possible. Positive~%# jerk indicates a desired absolute rate of acceleration change in~%# either direction (increasing or decreasing).~%#~%float32 speed                   # desired forward speed (m/s)~%float32 acceleration            # desired acceleration (m/s^2)~%float32 jerk                    # desired jerk (m/s^3)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VehicleState)))
  "Returns full string definition for message of type 'VehicleState"
  (cl:format cl:nil "Header header~%ackermann_msgs/AckermannDrive drive	# Look for ackermann_msgs at ros wiki~%int32 engine_speed					# Engine revolution-counter [range 0 to 10240 rpm]~%float64 throttle					# Throttle pedal position [range: 0 to 99.96%]~%int32 car_gear 						# Gear selection [range: -1 (reverse), 0 (neutral) to 5 (fifth)]~%int32 brake							# Brake active (1) or inactive (0)~%int32 handbrake						# Handbrake active (1) or inactive (0) ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: ackermann_msgs/AckermannDrive~%## Driving command for a car-like vehicle using Ackermann steering.~%#  $Id$~%~%# Assumes Ackermann front-wheel steering. The left and right front~%# wheels are generally at different angles. To simplify, the commanded~%# angle corresponds to the yaw of a virtual wheel located at the~%# center of the front axle, like on a tricycle.  Positive yaw is to~%# the left. (This is *not* the angle of the steering wheel inside the~%# passenger compartment.)~%#~%# Zero steering angle velocity means change the steering angle as~%# quickly as possible. Positive velocity indicates a desired absolute~%# rate of change either left or right. The controller tries not to~%# exceed this limit in either direction, but sometimes it might.~%#~%float32 steering_angle          # desired virtual angle (radians)~%float32 steering_angle_velocity # desired rate of change (radians/s)~%~%# Drive at requested speed, acceleration and jerk (the 1st, 2nd and~%# 3rd derivatives of position). All are measured at the vehicle's~%# center of rotation, typically the center of the rear axle. The~%# controller tries not to exceed these limits in either direction, but~%# sometimes it might.~%#~%# Speed is the desired scalar magnitude of the velocity vector.~%# Direction is forward unless the sign is negative, indicating reverse.~%#~%# Zero acceleration means change speed as quickly as~%# possible. Positive acceleration indicates a desired absolute~%# magnitude; that includes deceleration.~%#~%# Zero jerk means change acceleration as quickly as possible. Positive~%# jerk indicates a desired absolute rate of acceleration change in~%# either direction (increasing or decreasing).~%#~%float32 speed                   # desired forward speed (m/s)~%float32 acceleration            # desired acceleration (m/s^2)~%float32 jerk                    # desired jerk (m/s^3)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VehicleState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'drive))
     4
     8
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VehicleState>))
  "Converts a ROS message object to a list"
  (cl:list 'VehicleState
    (cl:cons ':header (header msg))
    (cl:cons ':drive (drive msg))
    (cl:cons ':engine_speed (engine_speed msg))
    (cl:cons ':throttle (throttle msg))
    (cl:cons ':car_gear (car_gear msg))
    (cl:cons ':brake (brake msg))
    (cl:cons ':handbrake (handbrake msg))
))
