; Auto-generated. Do not edit!


(cl:in-package amore-msg)


;//! \htmlinclude ned_buoy.msg.html

(cl:defclass <ned_buoy> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (id
    :reader id
    :initarg :id
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass ned_buoy (<ned_buoy>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ned_buoy>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ned_buoy)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amore-msg:<ned_buoy> is deprecated: use amore-msg:ned_buoy instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <ned_buoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:position-val is deprecated.  Use amore-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ned_buoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:id-val is deprecated.  Use amore-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ned_buoy>) ostream)
  "Serializes a message object of type '<ned_buoy>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'id) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ned_buoy>) istream)
  "Deserializes a message object of type '<ned_buoy>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'id) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ned_buoy>)))
  "Returns string type for a message object of type '<ned_buoy>"
  "amore/ned_buoy")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ned_buoy)))
  "Returns string type for a message object of type 'ned_buoy"
  "amore/ned_buoy")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ned_buoy>)))
  "Returns md5sum for a message object of type '<ned_buoy>"
  "790dcb77cf1dd99c8074cf037e8fb43c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ned_buoy)))
  "Returns md5sum for a message object of type 'ned_buoy"
  "790dcb77cf1dd99c8074cf037e8fb43c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ned_buoy>)))
  "Returns full string definition for message of type '<ned_buoy>"
  (cl:format cl:nil "geometry_msgs/Point position~%std_msgs/String id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ned_buoy)))
  "Returns full string definition for message of type 'ned_buoy"
  (cl:format cl:nil "geometry_msgs/Point position~%std_msgs/String id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ned_buoy>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ned_buoy>))
  "Converts a ROS message object to a list"
  (cl:list 'ned_buoy
    (cl:cons ':position (position msg))
    (cl:cons ':id (id msg))
))
