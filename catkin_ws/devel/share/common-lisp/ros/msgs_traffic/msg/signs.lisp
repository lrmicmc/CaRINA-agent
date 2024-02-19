; Auto-generated. Do not edit!


(cl:in-package msgs_traffic-msg)


;//! \htmlinclude signs.msg.html

(cl:defclass <signs> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass signs (<signs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <signs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'signs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msgs_traffic-msg:<signs> is deprecated: use msgs_traffic-msg:signs instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<signs>)))
    "Constants for message type '<signs>"
  '((:UNKNOWN . 0)
    (:STOP . 1)
    (:SPEED_LIMIT . 2)
    (:PEDESTRIAN_CROSSING . 3)
    (:SPEED_BUMP . 4)
    (:TRAFFIC_LIGHT . 5))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'signs)))
    "Constants for message type 'signs"
  '((:UNKNOWN . 0)
    (:STOP . 1)
    (:SPEED_LIMIT . 2)
    (:PEDESTRIAN_CROSSING . 3)
    (:SPEED_BUMP . 4)
    (:TRAFFIC_LIGHT . 5))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <signs>) ostream)
  "Serializes a message object of type '<signs>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <signs>) istream)
  "Deserializes a message object of type '<signs>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<signs>)))
  "Returns string type for a message object of type '<signs>"
  "msgs_traffic/signs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'signs)))
  "Returns string type for a message object of type 'signs"
  "msgs_traffic/signs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<signs>)))
  "Returns md5sum for a message object of type '<signs>"
  "e583dc6ff2378238d65c0f560909a609")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'signs)))
  "Returns md5sum for a message object of type 'signs"
  "e583dc6ff2378238d65c0f560909a609")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<signs>)))
  "Returns full string definition for message of type '<signs>"
  (cl:format cl:nil "#list of all traffic signs~%~%uint8 UNKNOWN             = 0~%uint8 STOP                = 1~%uint8 SPEED_LIMIT         = 2~%uint8 PEDESTRIAN_CROSSING = 3~%uint8 SPEED_BUMP          = 4~%uint8 TRAFFIC_LIGHT       = 5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'signs)))
  "Returns full string definition for message of type 'signs"
  (cl:format cl:nil "#list of all traffic signs~%~%uint8 UNKNOWN             = 0~%uint8 STOP                = 1~%uint8 SPEED_LIMIT         = 2~%uint8 PEDESTRIAN_CROSSING = 3~%uint8 SPEED_BUMP          = 4~%uint8 TRAFFIC_LIGHT       = 5~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <signs>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <signs>))
  "Converts a ROS message object to a list"
  (cl:list 'signs
))
