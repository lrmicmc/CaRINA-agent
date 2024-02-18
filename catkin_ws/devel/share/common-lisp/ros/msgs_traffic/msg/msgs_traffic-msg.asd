
(cl:in-package :asdf)

(defsystem "msgs_traffic-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "TrafficSign" :depends-on ("_package_TrafficSign"))
    (:file "_package_TrafficSign" :depends-on ("_package"))
    (:file "TrafficSignArray" :depends-on ("_package_TrafficSignArray"))
    (:file "_package_TrafficSignArray" :depends-on ("_package"))
    (:file "signs" :depends-on ("_package_signs"))
    (:file "_package_signs" :depends-on ("_package"))
    (:file "traffic_light" :depends-on ("_package_traffic_light"))
    (:file "_package_traffic_light" :depends-on ("_package"))
  ))