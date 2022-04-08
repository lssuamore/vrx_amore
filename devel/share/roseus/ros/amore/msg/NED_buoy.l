;; Auto-generated. Do not edit!


(when (boundp 'amore::NED_buoy)
  (if (not (find-package "AMORE"))
    (make-package "AMORE"))
  (shadow 'NED_buoy (find-package "AMORE")))
(unless (find-package "AMORE::NED_BUOY")
  (make-package "AMORE::NED_BUOY"))

(in-package "ROS")
;;//! \htmlinclude NED_buoy.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass amore::NED_buoy
  :super ros::object
  :slots (_position _id ))

(defmethod amore::NED_buoy
  (:init
   (&key
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:id __id) "")
    )
   (send-super :init)
   (setq _position __position)
   (setq _id (string __id))
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; string _id
    4 (length _id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get amore::NED_buoy :md5sum-) "b5a6f8471e86877e93afa9bcacce1774")
(setf (get amore::NED_buoy :datatype-) "amore/NED_buoy")
(setf (get amore::NED_buoy :definition-)
      "geometry_msgs/Point position
string id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :amore/NED_buoy "b5a6f8471e86877e93afa9bcacce1774")


