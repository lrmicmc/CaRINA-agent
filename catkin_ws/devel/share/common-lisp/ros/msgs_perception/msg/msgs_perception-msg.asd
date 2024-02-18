
(cl:in-package :asdf)

(defsystem "msgs_perception-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BoundingBox" :depends-on ("_package_BoundingBox"))
    (:file "_package_BoundingBox" :depends-on ("_package"))
    (:file "BoundingBoxArray" :depends-on ("_package_BoundingBoxArray"))
    (:file "_package_BoundingBoxArray" :depends-on ("_package"))
    (:file "Obstacle" :depends-on ("_package_Obstacle"))
    (:file "_package_Obstacle" :depends-on ("_package"))
    (:file "ObstacleArray" :depends-on ("_package_ObstacleArray"))
    (:file "_package_ObstacleArray" :depends-on ("_package"))
    (:file "StereoCloudImage" :depends-on ("_package_StereoCloudImage"))
    (:file "_package_StereoCloudImage" :depends-on ("_package"))
  ))