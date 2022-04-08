; Auto-generated. Do not edit!


(cl:in-package amore-msg)


;//! \htmlinclude NED_buoy.msg.html

(cl:defclass <NED_buoy> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (id
    :reader id
    :initarg :id
    :type cl:string
    :initform ""))
)

(cl:defclass NED_buoy (<NED_buoy>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NED_buoy>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NED_buoy)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amore-msg:<NED_buoy> is deprecated: use amore-msg:NED_buoy instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <NED_buoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:position-val is deprecated.  Use amore-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <NED_buoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:id-val is deprecated.  Use amore-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NED_buoy>) ostream)
  "Serializes a message object of type '<NED_buoy>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NED_buoy>) istream)
  "Deserializes a message object of type '<NED_buoy>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NED_buoy>)))
  "Returns string type for a message object of type '<NED_buoy>"
  "amore/NED_buoy")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NED_buoy)))
  "Returns string type for a message object of type 'NED_buoy"
  "amore/NED_buoy")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NED_buoy>)))
  "Returns md5sum for a message object of type '<NED_buoy>"
  "b5a6f8471e86877e93afa9bcacce1774")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NED_buoy)))
  "Returns md5sum for a message object of type 'NED_buoy"
  "b5a6f8471e86877e93afa9bcacce1774")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NED_buoy>)))
  "Returns full string definition for message of type '<NED_buoy>"
  (cl:format cl:nil "geometry_msgs/Point position~%string id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NED_buoy)))
  "Returns full string definition for message of type 'NED_buoy"
  (cl:format cl:nil "geometry_msgs/Point position~%string id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NED_buoy>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     4 (cl:length (cl:slot-value msg 'id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NED_buoy>))
  "Converts a ROS message object to a list"
  (cl:list 'NED_buoy
    (cl:cons ':position (position msg))
    (cl:cons ':id (id msg))
))
