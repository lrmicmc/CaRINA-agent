; Auto-generated. Do not edit!


(cl:in-package msgs_perception-msg)


;//! \htmlinclude BoundingBox.msg.html

(cl:defclass <BoundingBox> (roslisp-msg-protocol:ros-message)
  ((classe
    :reader classe
    :initarg :classe
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (p1
    :reader p1
    :initarg :p1
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (p2
    :reader p2
    :initarg :p2
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (p3
    :reader p3
    :initarg :p3
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (p4
    :reader p4
    :initarg :p4
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (probability
    :reader probability
    :initarg :probability
    :type cl:float
    :initform 0.0))
)

(cl:defclass BoundingBox (<BoundingBox>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BoundingBox>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BoundingBox)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msgs_perception-msg:<BoundingBox> is deprecated: use msgs_perception-msg:BoundingBox instead.")))

(cl:ensure-generic-function 'classe-val :lambda-list '(m))
(cl:defmethod classe-val ((m <BoundingBox>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_perception-msg:classe-val is deprecated.  Use msgs_perception-msg:classe instead.")
  (classe m))

(cl:ensure-generic-function 'p1-val :lambda-list '(m))
(cl:defmethod p1-val ((m <BoundingBox>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_perception-msg:p1-val is deprecated.  Use msgs_perception-msg:p1 instead.")
  (p1 m))

(cl:ensure-generic-function 'p2-val :lambda-list '(m))
(cl:defmethod p2-val ((m <BoundingBox>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_perception-msg:p2-val is deprecated.  Use msgs_perception-msg:p2 instead.")
  (p2 m))

(cl:ensure-generic-function 'p3-val :lambda-list '(m))
(cl:defmethod p3-val ((m <BoundingBox>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_perception-msg:p3-val is deprecated.  Use msgs_perception-msg:p3 instead.")
  (p3 m))

(cl:ensure-generic-function 'p4-val :lambda-list '(m))
(cl:defmethod p4-val ((m <BoundingBox>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_perception-msg:p4-val is deprecated.  Use msgs_perception-msg:p4 instead.")
  (p4 m))

(cl:ensure-generic-function 'probability-val :lambda-list '(m))
(cl:defmethod probability-val ((m <BoundingBox>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msgs_perception-msg:probability-val is deprecated.  Use msgs_perception-msg:probability instead.")
  (probability m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BoundingBox>) ostream)
  "Serializes a message object of type '<BoundingBox>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'classe) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p3) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p4) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'probability))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BoundingBox>) istream)
  "Deserializes a message object of type '<BoundingBox>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'classe) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p3) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p4) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'probability) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BoundingBox>)))
  "Returns string type for a message object of type '<BoundingBox>"
  "msgs_perception/BoundingBox")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BoundingBox)))
  "Returns string type for a message object of type 'BoundingBox"
  "msgs_perception/BoundingBox")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BoundingBox>)))
  "Returns md5sum for a message object of type '<BoundingBox>"
  "7f559df2715f948d4d334146387c4e69")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BoundingBox)))
  "Returns md5sum for a message object of type 'BoundingBox"
  "7f559df2715f948d4d334146387c4e69")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BoundingBox>)))
  "Returns full string definition for message of type '<BoundingBox>"
  (cl:format cl:nil "std_msgs/String classe~%~%geometry_msgs/Point p1~%geometry_msgs/Point p2~%geometry_msgs/Point p3~%geometry_msgs/Point p4~%~%float64 probability~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BoundingBox)))
  "Returns full string definition for message of type 'BoundingBox"
  (cl:format cl:nil "std_msgs/String classe~%~%geometry_msgs/Point p1~%geometry_msgs/Point p2~%geometry_msgs/Point p3~%geometry_msgs/Point p4~%~%float64 probability~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BoundingBox>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'classe))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p3))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p4))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BoundingBox>))
  "Converts a ROS message object to a list"
  (cl:list 'BoundingBox
    (cl:cons ':classe (classe msg))
    (cl:cons ':p1 (p1 msg))
    (cl:cons ':p2 (p2 msg))
    (cl:cons ':p3 (p3 msg))
    (cl:cons ':p4 (p4 msg))
    (cl:cons ':probability (probability msg))
))
