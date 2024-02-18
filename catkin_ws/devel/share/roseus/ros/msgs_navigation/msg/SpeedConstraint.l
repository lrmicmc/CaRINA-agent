;; Auto-generated. Do not edit!


(when (boundp 'msgs_navigation::SpeedConstraint)
  (if (not (find-package "MSGS_NAVIGATION"))
    (make-package "MSGS_NAVIGATION"))
  (shadow 'SpeedConstraint (find-package "MSGS_NAVIGATION")))
(unless (find-package "MSGS_NAVIGATION::SPEEDCONSTRAINT")
  (make-package "MSGS_NAVIGATION::SPEEDCONSTRAINT"))

(in-package "ROS")
;;//! \htmlinclude SpeedConstraint.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass msgs_navigation::SpeedConstraint
  :super ros::object
  :slots (_header _speed _decrease_rate _reason ))

(defmethod msgs_navigation::SpeedConstraint
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:speed __speed) 0.0)
    ((:decrease_rate __decrease_rate) 0.0)
    ((:reason __reason) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _speed (float __speed))
   (setq _decrease_rate (float __decrease_rate))
   (setq _reason (string __reason))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:decrease_rate
   (&optional __decrease_rate)
   (if __decrease_rate (setq _decrease_rate __decrease_rate)) _decrease_rate)
  (:reason
   (&optional __reason)
   (if __reason (setq _reason __reason)) _reason)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _speed
    8
    ;; float64 _decrease_rate
    8
    ;; string _reason
    4 (length _reason)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _decrease_rate
       (sys::poke _decrease_rate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _reason
       (write-long (length _reason) s) (princ _reason s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _speed
     (setq _speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _decrease_rate
     (setq _decrease_rate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _reason
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reason (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get msgs_navigation::SpeedConstraint :md5sum-) "09819746c374e0d4aba5a41ddeab09cb")
(setf (get msgs_navigation::SpeedConstraint :datatype-) "msgs_navigation/SpeedConstraint")
(setf (get msgs_navigation::SpeedConstraint :definition-)
      "std_msgs/Header header
float64 speed
float64 decrease_rate
string reason



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



(provide :msgs_navigation/SpeedConstraint "09819746c374e0d4aba5a41ddeab09cb")


