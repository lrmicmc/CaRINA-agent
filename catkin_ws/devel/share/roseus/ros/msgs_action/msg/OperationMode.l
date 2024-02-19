;; Auto-generated. Do not edit!


(when (boundp 'msgs_action::OperationMode)
  (if (not (find-package "MSGS_ACTION"))
    (make-package "MSGS_ACTION"))
  (shadow 'OperationMode (find-package "MSGS_ACTION")))
(unless (find-package "MSGS_ACTION::OPERATIONMODE")
  (make-package "MSGS_ACTION::OPERATIONMODE"))

(in-package "ROS")
;;//! \htmlinclude OperationMode.msg.html


(intern "*OPERATIONAL*" (find-package "MSGS_ACTION::OPERATIONMODE"))
(shadow '*OPERATIONAL* (find-package "MSGS_ACTION::OPERATIONMODE"))
(defconstant msgs_action::OperationMode::*OPERATIONAL* 0)
(intern "*EMERGENCY*" (find-package "MSGS_ACTION::OPERATIONMODE"))
(shadow '*EMERGENCY* (find-package "MSGS_ACTION::OPERATIONMODE"))
(defconstant msgs_action::OperationMode::*EMERGENCY* 1)
(intern "*MANUAL_ACCELERATION*" (find-package "MSGS_ACTION::OPERATIONMODE"))
(shadow '*MANUAL_ACCELERATION* (find-package "MSGS_ACTION::OPERATIONMODE"))
(defconstant msgs_action::OperationMode::*MANUAL_ACCELERATION* 0)
(intern "*AUTO_ACCELERATION*" (find-package "MSGS_ACTION::OPERATIONMODE"))
(shadow '*AUTO_ACCELERATION* (find-package "MSGS_ACTION::OPERATIONMODE"))
(defconstant msgs_action::OperationMode::*AUTO_ACCELERATION* 2)
(intern "*MANUAL_STEERING*" (find-package "MSGS_ACTION::OPERATIONMODE"))
(shadow '*MANUAL_STEERING* (find-package "MSGS_ACTION::OPERATIONMODE"))
(defconstant msgs_action::OperationMode::*MANUAL_STEERING* 0)
(intern "*AUTO_STEERING*" (find-package "MSGS_ACTION::OPERATIONMODE"))
(shadow '*AUTO_STEERING* (find-package "MSGS_ACTION::OPERATIONMODE"))
(defconstant msgs_action::OperationMode::*AUTO_STEERING* 4)

(defun msgs_action::OperationMode-to-symbol (const)
  (cond
        ((= const 0) 'msgs_action::OperationMode::*operational*)
        ((= const 1) 'msgs_action::OperationMode::*emergency*)
        ((= const 0) 'msgs_action::OperationMode::*manual_acceleration*)
        ((= const 2) 'msgs_action::OperationMode::*auto_acceleration*)
        ((= const 0) 'msgs_action::OperationMode::*manual_steering*)
        ((= const 4) 'msgs_action::OperationMode::*auto_steering*)
        (t nil)))

(defclass msgs_action::OperationMode
  :super ros::object
  :slots (_value ))

(defmethod msgs_action::OperationMode
  (:init
   (&key
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _value (round __value))
   self)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; int32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _value
       (write-long _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _value
     (setq _value (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get msgs_action::OperationMode :md5sum-) "77d89d0ca02e0b8d1ca091934c6fbbea")
(setf (get msgs_action::OperationMode :datatype-) "msgs_action/OperationMode")
(setf (get msgs_action::OperationMode :definition-)
      "# ART Navigator behaviors (lower numbers have higher priority)
# $Id: Behavior.msg 996 2011-02-27 16:07:34Z jack.oquin $

#Emergency state
int32 operational = 0
int32 emergency = 1

#Acceleration state
int32 manual_acceleration = 0
int32 auto_acceleration = 2

#Steering state
int32 manual_steering = 0
int32 auto_steering = 4

int32 value

")



(provide :msgs_action/OperationMode "77d89d0ca02e0b8d1ca091934c6fbbea")


