; Auto-generated. Do not edit!


(cl:in-package amore-msg)


;//! \htmlinclude NED_buoy.msg.html

(cl:defclass <NED_buoy> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (quantity
    :reader quantity
    :initarg :quantity
    :type cl:integer
    :initform 0))
)

(cl:defclass NED_buoy (<NED_buoy>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NED_buoy>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NED_buoy)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amore-msg:<NED_buoy> is deprecated: use amore-msg:NED_buoy instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <NED_buoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:header-val is deprecated.  Use amore-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <NED_buoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:position-val is deprecated.  Use amore-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'quantity-val :lambda-list '(m))
(cl:defmethod quantity-val ((m <NED_buoy>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:quantity-val is deprecated.  Use amore-msg:quantity instead.")
  (quantity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NED_buoy>) ostream)
  "Serializes a message object of type '<NED_buoy>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let* ((signed (cl:slot-value msg 'quantity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NED_buoy>) istream)
  "Deserializes a message object of type '<NED_buoy>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'quantity) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
  "6eb72406b17b4b923602e9676657c80a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NED_buoy)))
  "Returns md5sum for a message object of type 'NED_buoy"
  "6eb72406b17b4b923602e9676657c80a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NED_buoy>)))
  "Returns full string definition for message of type '<NED_buoy>"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Point position~%int32 quantity~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NED_buoy)))
  "Returns full string definition for message of type 'NED_buoy"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Point position~%int32 quantity~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NED_buoy>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NED_buoy>))
  "Converts a ROS message object to a list"
  (cl:list 'NED_buoy
    (cl:cons ':header (header msg))
    (cl:cons ':position (position msg))
    (cl:cons ':quantity (quantity msg))
))
