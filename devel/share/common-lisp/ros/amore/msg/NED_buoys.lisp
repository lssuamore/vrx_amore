; Auto-generated. Do not edit!


(cl:in-package amore-msg)


;//! \htmlinclude NED_buoys.msg.html

(cl:defclass <NED_buoys> (roslisp-msg-protocol:ros-message)
  ((buoys
    :reader buoys
    :initarg :buoys
    :type (cl:vector amore-msg:NED_buoy)
   :initform (cl:make-array 0 :element-type 'amore-msg:NED_buoy :initial-element (cl:make-instance 'amore-msg:NED_buoy)))
   (quantity
    :reader quantity
    :initarg :quantity
    :type cl:integer
    :initform 0))
)

(cl:defclass NED_buoys (<NED_buoys>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NED_buoys>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NED_buoys)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amore-msg:<NED_buoys> is deprecated: use amore-msg:NED_buoys instead.")))

(cl:ensure-generic-function 'buoys-val :lambda-list '(m))
(cl:defmethod buoys-val ((m <NED_buoys>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:buoys-val is deprecated.  Use amore-msg:buoys instead.")
  (buoys m))

(cl:ensure-generic-function 'quantity-val :lambda-list '(m))
(cl:defmethod quantity-val ((m <NED_buoys>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:quantity-val is deprecated.  Use amore-msg:quantity instead.")
  (quantity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NED_buoys>) ostream)
  "Serializes a message object of type '<NED_buoys>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'buoys))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'buoys))
  (cl:let* ((signed (cl:slot-value msg 'quantity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NED_buoys>) istream)
  "Deserializes a message object of type '<NED_buoys>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'buoys) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'buoys)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'amore-msg:NED_buoy))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'quantity) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NED_buoys>)))
  "Returns string type for a message object of type '<NED_buoys>"
  "amore/NED_buoys")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NED_buoys)))
  "Returns string type for a message object of type 'NED_buoys"
  "amore/NED_buoys")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NED_buoys>)))
  "Returns md5sum for a message object of type '<NED_buoys>"
  "770b53fd7910d5fb852692f8fc415cee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NED_buoys)))
  "Returns md5sum for a message object of type 'NED_buoys"
  "770b53fd7910d5fb852692f8fc415cee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NED_buoys>)))
  "Returns full string definition for message of type '<NED_buoys>"
  (cl:format cl:nil "amore/NED_buoy[] buoys~%int32 quantity~%~%================================================================================~%MSG: amore/NED_buoy~%geometry_msgs/Point position~%string id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NED_buoys)))
  "Returns full string definition for message of type 'NED_buoys"
  (cl:format cl:nil "amore/NED_buoy[] buoys~%int32 quantity~%~%================================================================================~%MSG: amore/NED_buoy~%geometry_msgs/Point position~%string id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NED_buoys>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'buoys) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NED_buoys>))
  "Converts a ROS message object to a list"
  (cl:list 'NED_buoys
    (cl:cons ':buoys (buoys msg))
    (cl:cons ':quantity (quantity msg))
))
