;; Auto-generated. Do not edit!


(when (boundp 'msgs_action::VehicleState)
  (if (not (find-package "MSGS_ACTION"))
    (make-package "MSGS_ACTION"))
  (shadow 'VehicleState (find-package "MSGS_ACTION")))
(unless (find-package "MSGS_ACTION::VEHICLESTATE")
  (make-package "MSGS_ACTION::VEHICLESTATE"))

(in-package "ROS")
;;//! \htmlinclude VehicleState.msg.html
(if (not (find-package "ACKERMANN_MSGS"))
  (ros::roseus-add-msgs "ackermann_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass msgs_action::VehicleState
  :super ros::object
  :slots (_header _drive _engine_speed _throttle _car_gear _brake _handbrake ))

(defmethod msgs_action::VehicleState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drive __drive) (instance ackermann_msgs::AckermannDrive :init))
    ((:engine_speed __engine_speed) 0)
    ((:throttle __throttle) 0.0)
    ((:car_gear __car_gear) 0)
    ((:brake __brake) 0)
    ((:handbrake __handbrake) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drive __drive)
   (setq _engine_speed (round __engine_speed))
   (setq _throttle (float __throttle))
   (setq _car_gear (round __car_gear))
   (setq _brake (round __brake))
   (setq _handbrake (round __handbrake))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:drive
   (&rest __drive)
   (if (keywordp (car __drive))
       (send* _drive __drive)
     (progn
       (if __drive (setq _drive (car __drive)))
       _drive)))
  (:engine_speed
   (&optional __engine_speed)
   (if __engine_speed (setq _engine_speed __engine_speed)) _engine_speed)
  (:throttle
   (&optional __throttle)
   (if __throttle (setq _throttle __throttle)) _throttle)
  (:car_gear
   (&optional __car_gear)
   (if __car_gear (setq _car_gear __car_gear)) _car_gear)
  (:brake
   (&optional __brake)
   (if __brake (setq _brake __brake)) _brake)
  (:handbrake
   (&optional __handbrake)
   (if __handbrake (setq _handbrake __handbrake)) _handbrake)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; ackermann_msgs/AckermannDrive _drive
    (send _drive :serialization-length)
    ;; int32 _engine_speed
    4
    ;; float64 _throttle
    8
    ;; int32 _car_gear
    4
    ;; int32 _brake
    4
    ;; int32 _handbrake
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; ackermann_msgs/AckermannDrive _drive
       (send _drive :serialize s)
     ;; int32 _engine_speed
       (write-long _engine_speed s)
     ;; float64 _throttle
       (sys::poke _throttle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _car_gear
       (write-long _car_gear s)
     ;; int32 _brake
       (write-long _brake s)
     ;; int32 _handbrake
       (write-long _handbrake s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; ackermann_msgs/AckermannDrive _drive
     (send _drive :deserialize buf ptr-) (incf ptr- (send _drive :serialization-length))
   ;; int32 _engine_speed
     (setq _engine_speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _throttle
     (setq _throttle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _car_gear
     (setq _car_gear (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _brake
     (setq _brake (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _handbrake
     (setq _handbrake (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get msgs_action::VehicleState :md5sum-) "d59fbb8f50f426518dc752cfb6444965")
(setf (get msgs_action::VehicleState :datatype-) "msgs_action/VehicleState")
(setf (get msgs_action::VehicleState :definition-)
      "Header header
ackermann_msgs/AckermannDrive drive	# Look for ackermann_msgs at ros wiki
int32 engine_speed					# Engine revolution-counter [range 0 to 10240 rpm]
float64 throttle					# Throttle pedal position [range: 0 to 99.96%]
int32 car_gear 						# Gear selection [range: -1 (reverse), 0 (neutral) to 5 (fifth)]
int32 brake							# Brake active (1) or inactive (0)
int32 handbrake						# Handbrake active (1) or inactive (0) 

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
MSG: ackermann_msgs/AckermannDrive
## Driving command for a car-like vehicle using Ackermann steering.
#  $Id$

# Assumes Ackermann front-wheel steering. The left and right front
# wheels are generally at different angles. To simplify, the commanded
# angle corresponds to the yaw of a virtual wheel located at the
# center of the front axle, like on a tricycle.  Positive yaw is to
# the left. (This is *not* the angle of the steering wheel inside the
# passenger compartment.)
#
# Zero steering angle velocity means change the steering angle as
# quickly as possible. Positive velocity indicates a desired absolute
# rate of change either left or right. The controller tries not to
# exceed this limit in either direction, but sometimes it might.
#
float32 steering_angle          # desired virtual angle (radians)
float32 steering_angle_velocity # desired rate of change (radians/s)

# Drive at requested speed, acceleration and jerk (the 1st, 2nd and
# 3rd derivatives of position). All are measured at the vehicle's
# center of rotation, typically the center of the rear axle. The
# controller tries not to exceed these limits in either direction, but
# sometimes it might.
#
# Speed is the desired scalar magnitude of the velocity vector.
# Direction is forward unless the sign is negative, indicating reverse.
#
# Zero acceleration means change speed as quickly as
# possible. Positive acceleration indicates a desired absolute
# magnitude; that includes deceleration.
#
# Zero jerk means change acceleration as quickly as possible. Positive
# jerk indicates a desired absolute rate of acceleration change in
# either direction (increasing or decreasing).
#
float32 speed                   # desired forward speed (m/s)
float32 acceleration            # desired acceleration (m/s^2)
float32 jerk                    # desired jerk (m/s^3)

")



(provide :msgs_action/VehicleState "d59fbb8f50f426518dc752cfb6444965")


