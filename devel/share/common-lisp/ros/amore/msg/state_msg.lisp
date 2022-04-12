; Auto-generated. Do not edit!


(cl:in-package amore-msg)


;//! \htmlinclude state_msg.msg.html

(cl:defclass <state_msg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32)))
)

(cl:defclass state_msg (<state_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <state_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'state_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amore-msg:<state_msg> is deprecated: use amore-msg:state_msg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <state_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:header-val is deprecated.  Use amore-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <state_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:state-val is deprecated.  Use amore-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <state_msg>) ostream)
  "Serializes a message object of type '<state_msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <state_msg>) istream)
  "Deserializes a message object of type '<state_msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<state_msg>)))
  "Returns string type for a message object of type '<state_msg>"
  "amore/state_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'state_msg)))
  "Returns string type for a message object of type 'state_msg"
  "amore/state_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<state_msg>)))
  "Returns md5sum for a message object of type '<state_msg>"
  "3471e556b0d295809f9950ffc322ab3d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'state_msg)))
  "Returns md5sum for a message object of type 'state_msg"
  "3471e556b0d295809f9950ffc322ab3d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<state_msg>)))
  "Returns full string definition for message of type '<state_msg>"
  (cl:format cl:nil "std_msgs/Header header~%std_msgs/Int32 state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'state_msg)))
  "Returns full string definition for message of type 'state_msg"
  (cl:format cl:nil "std_msgs/Header header~%std_msgs/Int32 state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <state_msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <state_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'state_msg
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
))
