
(cl:in-package :asdf)

(defsystem "msgs_navigation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :msgs_perception-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "EmergencyStop" :depends-on ("_package_EmergencyStop"))
    (:file "_package_EmergencyStop" :depends-on ("_package"))
    (:file "EmergencyStop" :depends-on ("_package_EmergencyStop"))
    (:file "_package_EmergencyStop" :depends-on ("_package"))
    (:file "GlobalPlan" :depends-on ("_package_GlobalPlan"))
    (:file "_package_GlobalPlan" :depends-on ("_package"))
    (:file "GlobalPlan" :depends-on ("_package_GlobalPlan"))
    (:file "_package_GlobalPlan" :depends-on ("_package"))
    (:file "Path" :depends-on ("_package_Path"))
    (:file "_package_Path" :depends-on ("_package"))
    (:file "Path" :depends-on ("_package_Path"))
    (:file "_package_Path" :depends-on ("_package"))
    (:file "SpeedConstraint" :depends-on ("_package_SpeedConstraint"))
    (:file "_package_SpeedConstraint" :depends-on ("_package"))
    (:file "SpeedConstraint" :depends-on ("_package_SpeedConstraint"))
    (:file "_package_SpeedConstraint" :depends-on ("_package"))
    (:file "TrajectoryError" :depends-on ("_package_TrajectoryError"))
    (:file "_package_TrajectoryError" :depends-on ("_package"))
    (:file "TrajectoryError" :depends-on ("_package_TrajectoryError"))
    (:file "_package_TrajectoryError" :depends-on ("_package"))
    (:file "TrajectoryPoint" :depends-on ("_package_TrajectoryPoint"))
    (:file "_package_TrajectoryPoint" :depends-on ("_package"))
    (:file "TrajectoryPoint" :depends-on ("_package_TrajectoryPoint"))
    (:file "_package_TrajectoryPoint" :depends-on ("_package"))
  ))