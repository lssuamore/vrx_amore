;; Auto-generated. Do not edit!


(when (boundp 'amore::NED_buoys)
  (if (not (find-package "AMORE"))
    (make-package "AMORE"))
  (shadow 'NED_buoys (find-package "AMORE")))
(unless (find-package "AMORE::NED_BUOYS")
  (make-package "AMORE::NED_BUOYS"))

(in-package "ROS")
;;//! \htmlinclude NED_buoys.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass amore::NED_buoys
  :super ros::object
  :slots (_buoys _quantity ))

(defmethod amore::NED_buoys
  (:init
   (&key
    ((:buoys __buoys) ())
    ((:quantity __quantity) 0)
    )
   (send-super :init)
   (setq _buoys __buoys)
   (setq _quantity (round __quantity))
   self)
  (:buoys
   (&rest __buoys)
   (if (keywordp (car __buoys))
       (send* _buoys __buoys)
     (progn
       (if __buoys (setq _buoys (car __buoys)))
       _buoys)))
  (:quantity
   (&optional __quantity)
   (if __quantity (setq _quantity __quantity)) _quantity)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PointStamped[] _buoys
    (apply #'+ (send-all _buoys :serialization-length)) 4
    ;; int32 _quantity
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PointStamped[] _buoys
     (write-long (length _buoys) s)
     (dolist (elem _buoys)
       (send elem :serialize s)
       )
     ;; int32 _quantity
       (write-long _quantity s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PointStamped[] _buoys
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _buoys (let (r) (dotimes (i n) (push (instance geometry_msgs::PointStamped :init) r)) r))
     (dolist (elem- _buoys)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int32 _quantity
     (setq _quantity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get amore::NED_buoys :md5sum-) "8674d58f3fb14856192d33f62b336838")
(setf (get amore::NED_buoys :datatype-) "amore/NED_buoys")
(setf (get amore::NED_buoys :definition-)
      "geometry_msgs/PointStamped[] buoys
int32 quantity

================================================================================
MSG: geometry_msgs/PointStamped
# This represents a Point with reference coordinate frame and timestamp
Header header
Point point

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

")



(provide :amore/NED_buoys "8674d58f3fb14856192d33f62b336838")


