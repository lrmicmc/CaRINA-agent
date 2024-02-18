;; Auto-generated. Do not edit!


(when (boundp 'msgs_navigation::TrajectoryError)
  (if (not (find-package "MSGS_NAVIGATION"))
    (make-package "MSGS_NAVIGATION"))
  (shadow 'TrajectoryError (find-package "MSGS_NAVIGATION")))
(unless (find-package "MSGS_NAVIGATION::TRAJECTORYERROR")
  (make-package "MSGS_NAVIGATION::TRAJECTORYERROR"))

(in-package "ROS")
;;//! \htmlinclude TrajectoryError.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*LATERAL*" (find-package "MSGS_NAVIGATION::TRAJECTORYERROR"))
(shadow '*LATERAL* (find-package "MSGS_NAVIGATION::TRAJECTORYERROR"))
(defconstant msgs_navigation::TrajectoryError::*LATERAL* 0)
(intern "*ANGULAR*" (find-package "MSGS_NAVIGATION::TRAJECTORYERROR"))
(shadow '*ANGULAR* (find-package "MSGS_NAVIGATION::TRAJECTORYERROR"))
(defconstant msgs_navigation::TrajectoryError::*ANGULAR* 1)
(intern "*LONGITUDINAL*" (find-package "MSGS_NAVIGATION::TRAJECTORYERROR"))
(shadow '*LONGITUDINAL* (find-package "MSGS_NAVIGATION::TRAJECTORYERROR"))
(defconstant msgs_navigation::TrajectoryError::*LONGITUDINAL* 2)
(intern "*KAPPA*" (find-package "MSGS_NAVIGATION::TRAJECTORYERROR"))
(shadow '*KAPPA* (find-package "MSGS_NAVIGATION::TRAJECTORYERROR"))
(defconstant msgs_navigation::TrajectoryError::*KAPPA* 3)

(defun msgs_navigation::TrajectoryError-to-symbol (const)
  (cond
        ((= const 0) 'msgs_navigation::TrajectoryError::*LATERAL*)
        ((= const 1) 'msgs_navigation::TrajectoryError::*ANGULAR*)
        ((= const 2) 'msgs_navigation::TrajectoryError::*LONGITUDINAL*)
        ((= const 3) 'msgs_navigation::TrajectoryError::*KAPPA*)
        (t nil)))

(defclass msgs_navigation::TrajectoryError
  :super ros::object
  :slots (_header _enable _lateral_error _angular_error _longitudinal_error _kappa_error ))

(defmethod msgs_navigation::TrajectoryError
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:enable __enable) (let (r) (dotimes (i 4) (push nil r)) r))
    ((:lateral_error __lateral_error) 0.0)
    ((:angular_error __angular_error) 0.0)
    ((:longitudinal_error __longitudinal_error) 0.0)
    ((:kappa_error __kappa_error) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _enable __enable)
   (setq _lateral_error (float __lateral_error))
   (setq _angular_error (float __angular_error))
   (setq _longitudinal_error (float __longitudinal_error))
   (setq _kappa_error (float __kappa_error))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:enable
   (&optional __enable)
   (if __enable (setq _enable __enable)) _enable)
  (:lateral_error
   (&optional __lateral_error)
   (if __lateral_error (setq _lateral_error __lateral_error)) _lateral_error)
  (:angular_error
   (&optional __angular_error)
   (if __angular_error (setq _angular_error __angular_error)) _angular_error)
  (:longitudinal_error
   (&optional __longitudinal_error)
   (if __longitudinal_error (setq _longitudinal_error __longitudinal_error)) _longitudinal_error)
  (:kappa_error
   (&optional __kappa_error)
   (if __kappa_error (setq _kappa_error __kappa_error)) _kappa_error)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool[4] _enable
    (* 1    4)
    ;; float64 _lateral_error
    8
    ;; float64 _angular_error
    8
    ;; float64 _longitudinal_error
    8
    ;; float64 _kappa_error
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool[4] _enable
     (dotimes (i 4)
       (if (elt _enable i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; float64 _lateral_error
       (sys::poke _lateral_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angular_error
       (sys::poke _angular_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitudinal_error
       (sys::poke _longitudinal_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _kappa_error
       (sys::poke _kappa_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool[4] _enable
   (dotimes (i (length _enable))
     (setf (elt _enable i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;; float64 _lateral_error
     (setq _lateral_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angular_error
     (setq _angular_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitudinal_error
     (setq _longitudinal_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _kappa_error
     (setq _kappa_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get msgs_navigation::TrajectoryError :md5sum-) "4965b313ef7d8cf00b86bfaf40b54c20")
(setf (get msgs_navigation::TrajectoryError :datatype-) "msgs_navigation/TrajectoryError")
(setf (get msgs_navigation::TrajectoryError :definition-)
      "std_msgs/Header header

bool[4] enable
uint8 LATERAL = 0
uint8 ANGULAR = 1
uint8 LONGITUDINAL = 2
uint8 KAPPA = 3

float64 lateral_error
float64 angular_error
float64 longitudinal_error
float64 kappa_error

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

")



(provide :msgs_navigation/TrajectoryError "4965b313ef7d8cf00b86bfaf40b54c20")


