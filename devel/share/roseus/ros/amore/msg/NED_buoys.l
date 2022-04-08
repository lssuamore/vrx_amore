;; Auto-generated. Do not edit!


(when (boundp 'amore::NED_buoys)
  (if (not (find-package "AMORE"))
    (make-package "AMORE"))
  (shadow 'NED_buoys (find-package "AMORE")))
(unless (find-package "AMORE::NED_BUOYS")
  (make-package "AMORE::NED_BUOYS"))

(in-package "ROS")
;;//! \htmlinclude NED_buoys.msg.html


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
    ;; amore/NED_buoy[] _buoys
    (apply #'+ (send-all _buoys :serialization-length)) 4
    ;; int32 _quantity
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; amore/NED_buoy[] _buoys
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
   ;; amore/NED_buoy[] _buoys
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _buoys (let (r) (dotimes (i n) (push (instance amore::NED_buoy :init) r)) r))
     (dolist (elem- _buoys)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int32 _quantity
     (setq _quantity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get amore::NED_buoys :md5sum-) "770b53fd7910d5fb852692f8fc415cee")
(setf (get amore::NED_buoys :datatype-) "amore/NED_buoys")
(setf (get amore::NED_buoys :definition-)
      "amore/NED_buoy[] buoys
int32 quantity

================================================================================
MSG: amore/NED_buoy
geometry_msgs/Point position
string id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :amore/NED_buoys "770b53fd7910d5fb852692f8fc415cee")


