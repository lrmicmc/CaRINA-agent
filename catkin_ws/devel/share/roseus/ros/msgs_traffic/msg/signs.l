;; Auto-generated. Do not edit!


(when (boundp 'msgs_traffic::signs)
  (if (not (find-package "MSGS_TRAFFIC"))
    (make-package "MSGS_TRAFFIC"))
  (shadow 'signs (find-package "MSGS_TRAFFIC")))
(unless (find-package "MSGS_TRAFFIC::SIGNS")
  (make-package "MSGS_TRAFFIC::SIGNS"))

(in-package "ROS")
;;//! \htmlinclude signs.msg.html


(intern "*UNKNOWN*" (find-package "MSGS_TRAFFIC::SIGNS"))
(shadow '*UNKNOWN* (find-package "MSGS_TRAFFIC::SIGNS"))
(defconstant msgs_traffic::signs::*UNKNOWN* 0)
(intern "*STOP*" (find-package "MSGS_TRAFFIC::SIGNS"))
(shadow '*STOP* (find-package "MSGS_TRAFFIC::SIGNS"))
(defconstant msgs_traffic::signs::*STOP* 1)
(intern "*SPEED_LIMIT*" (find-package "MSGS_TRAFFIC::SIGNS"))
(shadow '*SPEED_LIMIT* (find-package "MSGS_TRAFFIC::SIGNS"))
(defconstant msgs_traffic::signs::*SPEED_LIMIT* 2)
(intern "*PEDESTRIAN_CROSSING*" (find-package "MSGS_TRAFFIC::SIGNS"))
(shadow '*PEDESTRIAN_CROSSING* (find-package "MSGS_TRAFFIC::SIGNS"))
(defconstant msgs_traffic::signs::*PEDESTRIAN_CROSSING* 3)
(intern "*SPEED_BUMP*" (find-package "MSGS_TRAFFIC::SIGNS"))
(shadow '*SPEED_BUMP* (find-package "MSGS_TRAFFIC::SIGNS"))
(defconstant msgs_traffic::signs::*SPEED_BUMP* 4)
(intern "*TRAFFIC_LIGHT*" (find-package "MSGS_TRAFFIC::SIGNS"))
(shadow '*TRAFFIC_LIGHT* (find-package "MSGS_TRAFFIC::SIGNS"))
(defconstant msgs_traffic::signs::*TRAFFIC_LIGHT* 5)

(defun msgs_traffic::signs-to-symbol (const)
  (cond
        ((= const 0) 'msgs_traffic::signs::*UNKNOWN*)
        ((= const 1) 'msgs_traffic::signs::*STOP*)
        ((= const 2) 'msgs_traffic::signs::*SPEED_LIMIT*)
        ((= const 3) 'msgs_traffic::signs::*PEDESTRIAN_CROSSING*)
        ((= const 4) 'msgs_traffic::signs::*SPEED_BUMP*)
        ((= const 5) 'msgs_traffic::signs::*TRAFFIC_LIGHT*)
        (t nil)))

(defclass msgs_traffic::signs
  :super ros::object
  :slots ())

(defmethod msgs_traffic::signs
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get msgs_traffic::signs :md5sum-) "e583dc6ff2378238d65c0f560909a609")
(setf (get msgs_traffic::signs :datatype-) "msgs_traffic/signs")
(setf (get msgs_traffic::signs :definition-)
      "#list of all traffic signs

uint8 UNKNOWN             = 0
uint8 STOP                = 1
uint8 SPEED_LIMIT         = 2
uint8 PEDESTRIAN_CROSSING = 3
uint8 SPEED_BUMP          = 4
uint8 TRAFFIC_LIGHT       = 5

")



(provide :msgs_traffic/signs "e583dc6ff2378238d65c0f560909a609")


