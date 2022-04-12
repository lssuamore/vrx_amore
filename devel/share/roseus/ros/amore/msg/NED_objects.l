;; Auto-generated. Do not edit!


(when (boundp 'amore::NED_objects)
  (if (not (find-package "AMORE"))
    (make-package "AMORE"))
  (shadow 'NED_objects (find-package "AMORE")))
(unless (find-package "AMORE::NED_OBJECTS")
  (make-package "AMORE::NED_OBJECTS"))

(in-package "ROS")
;;//! \htmlinclude NED_objects.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass amore::NED_objects
  :super ros::object
  :slots (_objects _quantity ))

(defmethod amore::NED_objects
  (:init
   (&key
    ((:objects __objects) ())
    ((:quantity __quantity) 0)
    )
   (send-super :init)
   (setq _objects __objects)
   (setq _quantity (round __quantity))
   self)
  (:objects
   (&rest __objects)
   (if (keywordp (car __objects))
       (send* _objects __objects)
     (progn
       (if __objects (setq _objects (car __objects)))
       _objects)))
  (:quantity
   (&optional __quantity)
   (if __quantity (setq _quantity __quantity)) _quantity)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PointStamped[] _objects
    (apply #'+ (send-all _objects :serialization-length)) 4
    ;; int32 _quantity
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PointStamped[] _objects
     (write-long (length _objects) s)
     (dolist (elem _objects)
       (send elem :serialize s)
       )
     ;; int32 _quantity
       (write-long _quantity s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PointStamped[] _objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _objects (let (r) (dotimes (i n) (push (instance geometry_msgs::PointStamped :init) r)) r))
     (dolist (elem- _objects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int32 _quantity
     (setq _quantity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get amore::NED_objects :md5sum-) "228b5149963834c6d0e0916fa060e08d")
(setf (get amore::NED_objects :datatype-) "amore/NED_objects")
(setf (get amore::NED_objects :definition-)
      "geometry_msgs/PointStamped[] objects
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



(provide :amore/NED_objects "228b5149963834c6d0e0916fa060e08d")


