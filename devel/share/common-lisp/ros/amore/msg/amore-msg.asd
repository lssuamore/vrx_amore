
(cl:in-package :asdf)

(defsystem "amore-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "NED_buoys" :depends-on ("_package_NED_buoys"))
    (:file "_package_NED_buoys" :depends-on ("_package"))
    (:file "NED_waypoints" :depends-on ("_package_NED_waypoints"))
    (:file "_package_NED_waypoints" :depends-on ("_package"))
    (:file "state_msg" :depends-on ("_package_state_msg"))
    (:file "_package_state_msg" :depends-on ("_package"))
    (:file "usv_pose_msg" :depends-on ("_package_usv_pose_msg"))
    (:file "_package_usv_pose_msg" :depends-on ("_package"))
  ))