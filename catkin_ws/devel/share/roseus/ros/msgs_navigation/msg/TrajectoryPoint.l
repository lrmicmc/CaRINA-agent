;; Auto-generated. Do not edit!


(when (boundp 'msgs_navigation::TrajectoryPoint)
  (if (not (find-package "MSGS_NAVIGATION"))
    (make-package "MSGS_NAVIGATION"))
  (shadow 'TrajectoryPoint (find-package "MSGS_NAVIGATION")))
(unless (find-package "MSGS_NAVIGATION::TRAJECTORYPOINT")
  (make-package "MSGS_NAVIGATION::TRAJECTORYPOINT"))

(in-package "ROS")
;;//! \htmlinclude TrajectoryPoint.msg.html


(intern "*X*" (find-package "MSGS_NAVIGATION::TRAJECTORYPOINT"))
(shadow '*X* (find-package "MSGS_NAVIGATION::TRAJECTORYPOINT"))
(defconstant msgs_navigation::TrajectoryPoint::*X* 0)
(intern "*Y*" (find-package "MSGS_NAVIGATION::TRAJECTORYPOINT"))
(shadow '*Y* (find-package "MSGS_NAVIGATION::TRAJECTORYPOINT"))
(defconstant msgs_navigation::TrajectoryPoint::*Y* 1)
(intern "*KAPPA*" (find-package "MSGS_NAVIGATION::TRAJECTORYPOINT"))
(shadow '*KAPPA* (find-package "MSGS_NAVIGATION::TRAJECTORYPOINT"))
(defconstant msgs_navigation::TrajectoryPoint::*KAPPA* 2)
(intern "*ANGLE*" (find-package "MSGS_NAVIGATION::TRAJECTORYPOINT"))
(shadow '*ANGLE* (find-package "MSGS_NAVIGATION::TRAJECTORYPOINT"))
(defconstant msgs_navigation::TrajectoryPoint::*ANGLE* 3)
(intern "*LENGTH*" (find-package "MSGS_NAVIGATION::TRAJECTORYPOINT"))
(shadow '*LENGTH* (find-package "MSGS_NAVIGATION::TRAJECTORYPOINT"))
(defconstant msgs_navigation::TrajectoryPoint::*LENGTH* 4)
(intern "*SPEED*" (find-package "MSGS_NAVIGATION::TRAJECTORYPOINT"))
(shadow '*SPEED* (find-package "MSGS_NAVIGATION::TRAJECTORYPOINT"))
(defconstant msgs_navigation::TrajectoryPoint::*SPEED* 5)
(intern "*KAPPA_DERIVATIVE*" (find-package "MSGS_NAVIGATION::TRAJECTORYPOINT"))
(shadow '*KAPPA_DERIVATIVE* (find-package "MSGS_NAVIGATION::TRAJECTORYPOINT"))
(defconstant msgs_navigation::TrajectoryPoint::*KAPPA_DERIVATIVE* 6)

(defun msgs_navigation::TrajectoryPoint-to-symbol (const)
  (cond
        ((= const 0) 'msgs_navigation::TrajectoryPoint::*X*)
        ((= const 1) 'msgs_navigation::TrajectoryPoint::*Y*)
        ((= const 2) 'msgs_navigation::TrajectoryPoint::*KAPPA*)
        ((= const 3) 'msgs_navigation::TrajectoryPoint::*ANGLE*)
        ((= const 4) 'msgs_navigation::TrajectoryPoint::*LENGTH*)
        ((= const 5) 'msgs_navigation::TrajectoryPoint::*SPEED*)
        ((= const 6) 'msgs_navigation::TrajectoryPoint::*KAPPA_DERIVATIVE*)
        (t nil)))

(defclass msgs_navigation::TrajectoryPoint
  :super ros::object
  :slots (_point _point_number _end_track ))

(defmethod msgs_navigation::TrajectoryPoint
  (:init
   (&key
    ((:point __point) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:point_number __point_number) 0.0)
    ((:end_track __end_track) nil)
    )
   (send-super :init)
   (setq _point __point)
   (setq _point_number (float __point_number))
   (setq _end_track __end_track)
   self)
  (:point
   (&optional __point)
   (if __point (setq _point __point)) _point)
  (:point_number
   (&optional __point_number)
   (if __point_number (setq _point_number __point_number)) _point_number)
  (:end_track
   (&optional (__end_track :null))
   (if (not (eq __end_track :null)) (setq _end_track __end_track)) _end_track)
  (:serialization-length
   ()
   (+
    ;; float64[] _point
    (* 8    (length _point)) 4
    ;; float64 _point_number
    8
    ;; bool _end_track
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _point
     (write-long (length _point) s)
     (dotimes (i (length _point))
       (sys::poke (elt _point i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _point_number
       (sys::poke _point_number (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _end_track
       (if _end_track (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _point
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _point (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _point i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _point_number
     (setq _point_number (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _end_track
     (setq _end_track (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get msgs_navigation::TrajectoryPoint :md5sum-) "6e04038e6f3b9fe907134b6ecbe58bb7")
(setf (get msgs_navigation::TrajectoryPoint :datatype-) "msgs_navigation/TrajectoryPoint")
(setf (get msgs_navigation::TrajectoryPoint :definition-)
      "# Clothoid data

float64[] point

uint8 X = 0
uint8 Y = 1
uint8 KAPPA = 2
uint8 ANGLE = 3
uint8 LENGTH = 4
uint8 SPEED = 5
uint8 KAPPA_DERIVATIVE = 6

float64 point_number
bool end_track

")



(provide :msgs_navigation/TrajectoryPoint "6e04038e6f3b9fe907134b6ecbe58bb7")


