
(cl:in-package :asdf)

(defsystem "msgs_mapping-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "HDMap" :depends-on ("_package_HDMap"))
    (:file "_package_HDMap" :depends-on ("_package"))
  ))