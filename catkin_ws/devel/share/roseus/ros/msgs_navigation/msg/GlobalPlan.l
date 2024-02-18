;; Auto-generated. Do not edit!


(when (boundp 'msgs_navigation::GlobalPlan)
  (if (not (find-package "MSGS_NAVIGATION"))
    (make-package "MSGS_NAVIGATION"))
  (shadow 'GlobalPlan (find-package "MSGS_NAVIGATION")))
(unless (find-package "MSGS_NAVIGATION::GLOBALPLAN")
  (make-package "MSGS_NAVIGATION::GLOBALPLAN"))

(in-package "ROS")
;;//! \htmlinclude GlobalPlan.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*LANEFOLLOW*" (find-package "MSGS_NAVIGATION::GLOBALPLAN"))
(shadow '*LANEFOLLOW* (find-package "MSGS_NAVIGATION::GLOBALPLAN"))
(defconstant msgs_navigation::GlobalPlan::*LANEFOLLOW* 0)
(intern "*STRAIGHT*" (find-package "MSGS_NAVIGATION::GLOBALPLAN"))
(shadow '*STRAIGHT* (find-package "MSGS_NAVIGATION::GLOBALPLAN"))
(defconstant msgs_navigation::GlobalPlan::*STRAIGHT* 1)
(intern "*RIGHT*" (find-package "MSGS_NAVIGATION::GLOBALPLAN"))
(shadow '*RIGHT* (find-package "MSGS_NAVIGATION::GLOBALPLAN"))
(defconstant msgs_navigation::GlobalPlan::*RIGHT* 2)
(intern "*LEFT*" (find-package "MSGS_NAVIGATION::GLOBALPLAN"))
(shadow '*LEFT* (find-package "MSGS_NAVIGATION::GLOBALPLAN"))
(defconstant msgs_navigation::GlobalPlan::*LEFT* 3)
(intern "*CHANGELANELEFT*" (find-package "MSGS_NAVIGATION::GLOBALPLAN"))
(shadow '*CHANGELANELEFT* (find-package "MSGS_NAVIGATION::GLOBALPLAN"))
(defconstant msgs_navigation::GlobalPlan::*CHANGELANELEFT* 4)
(intern "*CHANGELANERIGHT*" (find-package "MSGS_NAVIGATION::GLOBALPLAN"))
(shadow '*CHANGELANERIGHT* (find-package "MSGS_NAVIGATION::GLOBALPLAN"))
(defconstant msgs_navigation::GlobalPlan::*CHANGELANERIGHT* 5)
(intern "*UNKNOWN*" (find-package "MSGS_NAVIGATION::GLOBALPLAN"))
(shadow '*UNKNOWN* (find-package "MSGS_NAVIGATION::GLOBALPLAN"))
(defconstant msgs_navigation::GlobalPlan::*UNKNOWN* 6)

(defun msgs_navigation::GlobalPlan-to-symbol (const)
  (cond
        ((= const 0) 'msgs_navigation::GlobalPlan::*LANEFOLLOW*)
        ((= const 1) 'msgs_navigation::GlobalPlan::*STRAIGHT*)
        ((= const 2) 'msgs_navigation::GlobalPlan::*RIGHT*)
        ((= const 3) 'msgs_navigation::GlobalPlan::*LEFT*)
        ((= const 4) 'msgs_navigation::GlobalPlan::*CHANGELANELEFT*)
        ((= const 5) 'msgs_navigation::GlobalPlan::*CHANGELANERIGHT*)
        ((= const 6) 'msgs_navigation::GlobalPlan::*UNKNOWN*)
        (t nil)))

(defclass msgs_navigation::GlobalPlan
  :super ros::object
  :slots (_header _points _road_options ))

(defmethod msgs_navigation::GlobalPlan
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:points __points) ())
    ((:road_options __road_options) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _points __points)
   (setq _road_options __road_options)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:road_options
   (&optional __road_options)
   (if __road_options (setq _road_options __road_options)) _road_options)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ;; int32[] _road_options
    (* 4    (length _road_options)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;; int32[] _road_options
     (write-long (length _road_options) s)
     (dotimes (i (length _road_options))
       (write-long (elt _road_options i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int32[] _road_options
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _road_options (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _road_options i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get msgs_navigation::GlobalPlan :md5sum-) "30bdb54aae74b679c913a2e5f20b612e")
(setf (get msgs_navigation::GlobalPlan :datatype-) "msgs_navigation/GlobalPlan")
(setf (get msgs_navigation::GlobalPlan :definition-)
      "std_msgs/Header header

geometry_msgs/Point[] points
int32[] road_options

uint8 LANEFOLLOW=0
uint8 STRAIGHT=1
uint8 RIGHT=2
uint8 LEFT=3
uint8 CHANGELANELEFT=4
uint8 CHANGELANERIGHT=5
uint8 UNKNOWN=6




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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :msgs_navigation/GlobalPlan "30bdb54aae74b679c913a2e5f20b612e")


