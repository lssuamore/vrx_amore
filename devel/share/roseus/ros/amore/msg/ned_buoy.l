;; Auto-generated. Do not edit!


(when (boundp 'amore::ned_buoy)
  (if (not (find-package "AMORE"))
    (make-package "AMORE"))
  (shadow 'ned_buoy (find-package "AMORE")))
(unless (find-package "AMORE::NED_BUOY")
  (make-package "AMORE::NED_BUOY"))

(in-package "ROS")
;;//! \htmlinclude ned_buoy.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass amore::ned_buoy
  :super ros::object
  :slots (_position _id ))

(defmethod amore::ned_buoy
  (:init
   (&key
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:id __id) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _position __position)
   (setq _id __id)
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:id
   (&rest __id)
   (if (keywordp (car __id))
       (send* _id __id)
     (progn
       (if __id (setq _id (car __id)))
       _id)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; std_msgs/String _id
    (send _id :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; std_msgs/String _id
       (send _id :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; std_msgs/String _id
     (send _id :deserialize buf ptr-) (incf ptr- (send _id :serialization-length))
   ;;
   self)
  )

(setf (get amore::ned_buoy :md5sum-) "790dcb77cf1dd99c8074cf037e8fb43c")
(setf (get amore::ned_buoy :datatype-) "amore/ned_buoy")
(setf (get amore::ned_buoy :definition-)
      "geometry_msgs/Point position
std_msgs/String id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: std_msgs/String
string data

")



(provide :amore/ned_buoy "790dcb77cf1dd99c8074cf037e8fb43c")


