; Auto-generated. Do not edit!


(cl:in-package amore-msg)


;//! \htmlinclude usv_pose_msg.msg.html

(cl:defclass <usv_pose_msg> (roslisp-msg-protocol:ros-message)
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
   (psi
    :reader psi
    :initarg :psi
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (latitude
    :reader latitude
    :initarg :latitude
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (longitude
    :reader longitude
    :initarg :longitude
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass usv_pose_msg (<usv_pose_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <usv_pose_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'usv_pose_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amore-msg:<usv_pose_msg> is deprecated: use amore-msg:usv_pose_msg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <usv_pose_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:header-val is deprecated.  Use amore-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <usv_pose_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:position-val is deprecated.  Use amore-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'psi-val :lambda-list '(m))
(cl:defmethod psi-val ((m <usv_pose_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:psi-val is deprecated.  Use amore-msg:psi instead.")
  (psi m))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <usv_pose_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:latitude-val is deprecated.  Use amore-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <usv_pose_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amore-msg:longitude-val is deprecated.  Use amore-msg:longitude instead.")
  (longitude m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <usv_pose_msg>) ostream)
  "Serializes a message object of type '<usv_pose_msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'psi) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'latitude) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'longitude) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <usv_pose_msg>) istream)
  "Deserializes a message object of type '<usv_pose_msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'psi) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'latitude) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'longitude) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<usv_pose_msg>)))
  "Returns string type for a message object of type '<usv_pose_msg>"
  "amore/usv_pose_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'usv_pose_msg)))
  "Returns string type for a message object of type 'usv_pose_msg"
  "amore/usv_pose_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<usv_pose_msg>)))
  "Returns md5sum for a message object of type '<usv_pose_msg>"
  "ec83d6a1d1a8805fa2cb78e46565b333")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'usv_pose_msg)))
  "Returns md5sum for a message object of type 'usv_pose_msg"
  "ec83d6a1d1a8805fa2cb78e46565b333")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<usv_pose_msg>)))
  "Returns full string definition for message of type '<usv_pose_msg>"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Point position~%std_msgs/Float64 psi~%std_msgs/Float64 latitude~%std_msgs/Float64 longitude~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'usv_pose_msg)))
  "Returns full string definition for message of type 'usv_pose_msg"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Point position~%std_msgs/Float64 psi~%std_msgs/Float64 latitude~%std_msgs/Float64 longitude~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <usv_pose_msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'psi))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'latitude))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'longitude))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <usv_pose_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'usv_pose_msg
    (cl:cons ':header (header msg))
    (cl:cons ':position (position msg))
    (cl:cons ':psi (psi msg))
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
))
