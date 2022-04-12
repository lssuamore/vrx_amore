;; Auto-generated. Do not edit!


(when (boundp 'amore::usv_pose_msg)
  (if (not (find-package "AMORE"))
    (make-package "AMORE"))
  (shadow 'usv_pose_msg (find-package "AMORE")))
(unless (find-package "AMORE::USV_POSE_MSG")
  (make-package "AMORE::USV_POSE_MSG"))

(in-package "ROS")
;;//! \htmlinclude usv_pose_msg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass amore::usv_pose_msg
  :super ros::object
  :slots (_header _position _psi _latitude _longitude ))

(defmethod amore::usv_pose_msg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:psi __psi) (instance std_msgs::Float64 :init))
    ((:latitude __latitude) (instance std_msgs::Float64 :init))
    ((:longitude __longitude) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _position __position)
   (setq _psi __psi)
   (setq _latitude __latitude)
   (setq _longitude __longitude)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:psi
   (&rest __psi)
   (if (keywordp (car __psi))
       (send* _psi __psi)
     (progn
       (if __psi (setq _psi (car __psi)))
       _psi)))
  (:latitude
   (&rest __latitude)
   (if (keywordp (car __latitude))
       (send* _latitude __latitude)
     (progn
       (if __latitude (setq _latitude (car __latitude)))
       _latitude)))
  (:longitude
   (&rest __longitude)
   (if (keywordp (car __longitude))
       (send* _longitude __longitude)
     (progn
       (if __longitude (setq _longitude (car __longitude)))
       _longitude)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; std_msgs/Float64 _psi
    (send _psi :serialization-length)
    ;; std_msgs/Float64 _latitude
    (send _latitude :serialization-length)
    ;; std_msgs/Float64 _longitude
    (send _longitude :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; std_msgs/Float64 _psi
       (send _psi :serialize s)
     ;; std_msgs/Float64 _latitude
       (send _latitude :serialize s)
     ;; std_msgs/Float64 _longitude
       (send _longitude :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; std_msgs/Float64 _psi
     (send _psi :deserialize buf ptr-) (incf ptr- (send _psi :serialization-length))
   ;; std_msgs/Float64 _latitude
     (send _latitude :deserialize buf ptr-) (incf ptr- (send _latitude :serialization-length))
   ;; std_msgs/Float64 _longitude
     (send _longitude :deserialize buf ptr-) (incf ptr- (send _longitude :serialization-length))
   ;;
   self)
  )

(setf (get amore::usv_pose_msg :md5sum-) "ec83d6a1d1a8805fa2cb78e46565b333")
(setf (get amore::usv_pose_msg :datatype-) "amore/usv_pose_msg")
(setf (get amore::usv_pose_msg :definition-)
      "std_msgs/Header header
geometry_msgs/Point position
std_msgs/Float64 psi
std_msgs/Float64 latitude
std_msgs/Float64 longitude

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: std_msgs/Float64
float64 data
")



(provide :amore/usv_pose_msg "ec83d6a1d1a8805fa2cb78e46565b333")


