;; Auto-generated. Do not edit!


(when (boundp 'msgs_perception::BoundingBox)
  (if (not (find-package "MSGS_PERCEPTION"))
    (make-package "MSGS_PERCEPTION"))
  (shadow 'BoundingBox (find-package "MSGS_PERCEPTION")))
(unless (find-package "MSGS_PERCEPTION::BOUNDINGBOX")
  (make-package "MSGS_PERCEPTION::BOUNDINGBOX"))

(in-package "ROS")
;;//! \htmlinclude BoundingBox.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass msgs_perception::BoundingBox
  :super ros::object
  :slots (_classe _p1 _p2 _p3 _p4 _probability ))

(defmethod msgs_perception::BoundingBox
  (:init
   (&key
    ((:classe __classe) (instance std_msgs::String :init))
    ((:p1 __p1) (instance geometry_msgs::Point :init))
    ((:p2 __p2) (instance geometry_msgs::Point :init))
    ((:p3 __p3) (instance geometry_msgs::Point :init))
    ((:p4 __p4) (instance geometry_msgs::Point :init))
    ((:probability __probability) 0.0)
    )
   (send-super :init)
   (setq _classe __classe)
   (setq _p1 __p1)
   (setq _p2 __p2)
   (setq _p3 __p3)
   (setq _p4 __p4)
   (setq _probability (float __probability))
   self)
  (:classe
   (&rest __classe)
   (if (keywordp (car __classe))
       (send* _classe __classe)
     (progn
       (if __classe (setq _classe (car __classe)))
       _classe)))
  (:p1
   (&rest __p1)
   (if (keywordp (car __p1))
       (send* _p1 __p1)
     (progn
       (if __p1 (setq _p1 (car __p1)))
       _p1)))
  (:p2
   (&rest __p2)
   (if (keywordp (car __p2))
       (send* _p2 __p2)
     (progn
       (if __p2 (setq _p2 (car __p2)))
       _p2)))
  (:p3
   (&rest __p3)
   (if (keywordp (car __p3))
       (send* _p3 __p3)
     (progn
       (if __p3 (setq _p3 (car __p3)))
       _p3)))
  (:p4
   (&rest __p4)
   (if (keywordp (car __p4))
       (send* _p4 __p4)
     (progn
       (if __p4 (setq _p4 (car __p4)))
       _p4)))
  (:probability
   (&optional __probability)
   (if __probability (setq _probability __probability)) _probability)
  (:serialization-length
   ()
   (+
    ;; std_msgs/String _classe
    (send _classe :serialization-length)
    ;; geometry_msgs/Point _p1
    (send _p1 :serialization-length)
    ;; geometry_msgs/Point _p2
    (send _p2 :serialization-length)
    ;; geometry_msgs/Point _p3
    (send _p3 :serialization-length)
    ;; geometry_msgs/Point _p4
    (send _p4 :serialization-length)
    ;; float64 _probability
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _classe
       (send _classe :serialize s)
     ;; geometry_msgs/Point _p1
       (send _p1 :serialize s)
     ;; geometry_msgs/Point _p2
       (send _p2 :serialize s)
     ;; geometry_msgs/Point _p3
       (send _p3 :serialize s)
     ;; geometry_msgs/Point _p4
       (send _p4 :serialize s)
     ;; float64 _probability
       (sys::poke _probability (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _classe
     (send _classe :deserialize buf ptr-) (incf ptr- (send _classe :serialization-length))
   ;; geometry_msgs/Point _p1
     (send _p1 :deserialize buf ptr-) (incf ptr- (send _p1 :serialization-length))
   ;; geometry_msgs/Point _p2
     (send _p2 :deserialize buf ptr-) (incf ptr- (send _p2 :serialization-length))
   ;; geometry_msgs/Point _p3
     (send _p3 :deserialize buf ptr-) (incf ptr- (send _p3 :serialization-length))
   ;; geometry_msgs/Point _p4
     (send _p4 :deserialize buf ptr-) (incf ptr- (send _p4 :serialization-length))
   ;; float64 _probability
     (setq _probability (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get msgs_perception::BoundingBox :md5sum-) "7f559df2715f948d4d334146387c4e69")
(setf (get msgs_perception::BoundingBox :datatype-) "msgs_perception/BoundingBox")
(setf (get msgs_perception::BoundingBox :definition-)
      "std_msgs/String classe

geometry_msgs/Point p1
geometry_msgs/Point p2
geometry_msgs/Point p3
geometry_msgs/Point p4

float64 probability

================================================================================
MSG: std_msgs/String
string data

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :msgs_perception/BoundingBox "7f559df2715f948d4d334146387c4e69")


