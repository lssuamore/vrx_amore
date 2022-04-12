; Auto-generated. Do not edit!


(cl:in-package amore-msg)


;//! \htmlinclude NED_acoustic.msg.html

(cl:defclass <NED_acoustic> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (quantity
    :reader quantity
    :initarg :quantity
    :type cl:integer
    :initform 0))
)

(cl:defclass NED_acoustic (<NED_acoustic>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NED_acoustic>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NED_acoustic)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amore-msg:<NED_acoustic> is deprecated: use amore-msg:NED_acoustic instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <NED_acoustic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:points-val is deprecated.  Use amore-msg:points instead.")
  (points m))

(cl:ensure-generic-function 'quantity-val :lambda-list '(m))
(cl:defmethod quantity-val ((m <NED_acoustic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:quantity-val is deprecated.  Use amore-msg:quantity instead.")
  (quantity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NED_acoustic>) ostream)
  "Serializes a message object of type '<NED_acoustic>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
  (cl:let* ((signed (cl:slot-value msg 'quantity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NED_acoustic>) istream)
  "Deserializes a message object of type '<NED_acoustic>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'quantity) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NED_acoustic>)))
  "Returns string type for a message object of type '<NED_acoustic>"
  "amore/NED_acoustic")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NED_acoustic)))
  "Returns string type for a message object of type 'NED_acoustic"
  "amore/NED_acoustic")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NED_acoustic>)))
  "Returns md5sum for a message object of type '<NED_acoustic>"
  "3ccc3a0c67805b14fa25891768bbbb3d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NED_acoustic)))
  "Returns md5sum for a message object of type 'NED_acoustic"
  "3ccc3a0c67805b14fa25891768bbbb3d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NED_acoustic>)))
  "Returns full string definition for message of type '<NED_acoustic>"
  (cl:format cl:nil "geometry_msgs/Point[] points~%int32 quantity~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NED_acoustic)))
  "Returns full string definition for message of type 'NED_acoustic"
  (cl:format cl:nil "geometry_msgs/Point[] points~%int32 quantity~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NED_acoustic>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NED_acoustic>))
  "Converts a ROS message object to a list"
  (cl:list 'NED_acoustic
    (cl:cons ':points (points msg))
    (cl:cons ':quantity (quantity msg))
))
