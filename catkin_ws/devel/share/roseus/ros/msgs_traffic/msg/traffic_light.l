;; Auto-generated. Do not edit!


(when (boundp 'msgs_traffic::traffic_light)
  (if (not (find-package "MSGS_TRAFFIC"))
    (make-package "MSGS_TRAFFIC"))
  (shadow 'traffic_light (find-package "MSGS_TRAFFIC")))
(unless (find-package "MSGS_TRAFFIC::TRAFFIC_LIGHT")
  (make-package "MSGS_TRAFFIC::TRAFFIC_LIGHT"))

(in-package "ROS")
;;//! \htmlinclude traffic_light.msg.html


(intern "*RED*" (find-package "MSGS_TRAFFIC::TRAFFIC_LIGHT"))
(shadow '*RED* (find-package "MSGS_TRAFFIC::TRAFFIC_LIGHT"))
(defconstant msgs_traffic::traffic_light::*RED* 0)
(intern "*GREEN*" (find-package "MSGS_TRAFFIC::TRAFFIC_LIGHT"))
(shadow '*GREEN* (find-package "MSGS_TRAFFIC::TRAFFIC_LIGHT"))
(defconstant msgs_traffic::traffic_light::*GREEN* 1)
(intern "*YELLOW*" (find-package "MSGS_TRAFFIC::TRAFFIC_LIGHT"))
(shadow '*YELLOW* (find-package "MSGS_TRAFFIC::TRAFFIC_LIGHT"))
(defconstant msgs_traffic::traffic_light::*YELLOW* 2)

(defun msgs_traffic::traffic_light-to-symbol (const)
  (cond
        ((= const 0) 'msgs_traffic::traffic_light::*RED*)
        ((= const 1) 'msgs_traffic::traffic_light::*GREEN*)
        ((= const 2) 'msgs_traffic::traffic_light::*YELLOW*)
        (t nil)))

(defclass msgs_traffic::traffic_light
  :super ros::object
  :slots (_color ))

(defmethod msgs_traffic::traffic_light
  (:init
   (&key
    ((:color __color) 0)
    )
   (send-super :init)
   (setq _color (round __color))
   self)
  (:color
   (&optional __color)
   (if __color (setq _color __color)) _color)
  (:serialization-length
   ()
   (+
    ;; uint8 _color
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _color
       (write-byte _color s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _color
     (setq _color (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get msgs_traffic::traffic_light :md5sum-) "41d97d57d4130b7bdbe8ee80a349b8af")
(setf (get msgs_traffic::traffic_light :datatype-) "msgs_traffic/traffic_light")
(setf (get msgs_traffic::traffic_light :definition-)
      "uint8 RED    = 0
uint8 GREEN  = 1
uint8 YELLOW = 2

uint8 color

")



(provide :msgs_traffic/traffic_light "41d97d57d4130b7bdbe8ee80a349b8af")


