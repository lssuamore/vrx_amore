;; Auto-generated. Do not edit!


(when (boundp 'amore::NED_waypoints)
  (if (not (find-package "AMORE"))
    (make-package "AMORE"))
  (shadow 'NED_waypoints (find-package "AMORE")))
(unless (find-package "AMORE::NED_WAYPOINTS")
  (make-package "AMORE::NED_WAYPOINTS"))

(in-package "ROS")
;;//! \htmlinclude NED_waypoints.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass amore::NED_waypoints
  :super ros::object
  :slots (_points _quantity ))

(defmethod amore::NED_waypoints
  (:init
   (&key
    ((:points __points) ())
    ((:quantity __quantity) 0)
    )
   (send-super :init)
   (setq _points __points)
   (setq _quantity (round __quantity))
   self)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:quantity
   (&optional __quantity)
   (if __quantity (setq _quantity __quantity)) _quantity)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ;; int32 _quantity
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;; int32 _quantity
       (write-long _quantity s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int32 _quantity
     (setq _quantity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get amore::NED_waypoints :md5sum-) "3ccc3a0c67805b14fa25891768bbbb3d")
(setf (get amore::NED_waypoints :datatype-) "amore/NED_waypoints")
(setf (get amore::NED_waypoints :definition-)
      "geometry_msgs/Point[] points
int32 quantity

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :amore/NED_waypoints "3ccc3a0c67805b14fa25891768bbbb3d")


