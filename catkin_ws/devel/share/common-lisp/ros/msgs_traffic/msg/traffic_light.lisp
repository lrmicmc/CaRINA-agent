; Auto-generated. Do not edit!


(cl:in-package msgs_traffic-msg)


;//! \htmlinclude traffic_light.msg.html

(cl:defclass <traffic_light> (roslisp-msg-protocol:ros-message)
  ((color
    :reader color
    :initarg :color
    :type cl:fixnum
    :initform 0))
)

(cl:defclass traffic_light (<traffic_light>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <traffic_light>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'traffic_light)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msgs_traffic-msg:<traffic_light> is deprecated: use msgs_traffic-msg:traffic_light instead.")))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <traffic_light>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_traffic-msg:color-val is deprecated.  Use msgs_traffic-msg:color instead.")
  (color m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<traffic_light>)))
    "Constants for message type '<traffic_light>"
  '((:RED . 0)
    (:GREEN . 1)
    (:YELLOW . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'traffic_light)))
    "Constants for message type 'traffic_light"
  '((:RED . 0)
    (:GREEN . 1)
    (:YELLOW . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <traffic_light>) ostream)
  "Serializes a message object of type '<traffic_light>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'color)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <traffic_light>) istream)
  "Deserializes a message object of type '<traffic_light>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'color)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<traffic_light>)))
  "Returns string type for a message object of type '<traffic_light>"
  "msgs_traffic/traffic_light")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'traffic_light)))
  "Returns string type for a message object of type 'traffic_light"
  "msgs_traffic/traffic_light")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<traffic_light>)))
  "Returns md5sum for a message object of type '<traffic_light>"
  "41d97d57d4130b7bdbe8ee80a349b8af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'traffic_light)))
  "Returns md5sum for a message object of type 'traffic_light"
  "41d97d57d4130b7bdbe8ee80a349b8af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<traffic_light>)))
  "Returns full string definition for message of type '<traffic_light>"
  (cl:format cl:nil "uint8 RED    = 0~%uint8 GREEN  = 1~%uint8 YELLOW = 2~%~%uint8 color~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'traffic_light)))
  "Returns full string definition for message of type 'traffic_light"
  (cl:format cl:nil "uint8 RED    = 0~%uint8 GREEN  = 1~%uint8 YELLOW = 2~%~%uint8 color~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <traffic_light>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <traffic_light>))
  "Converts a ROS message object to a list"
  (cl:list 'traffic_light
    (cl:cons ':color (color msg))
))
