
(cl:in-package :asdf)

(defsystem "amore-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "NED_acoustic" :depends-on ("_package_NED_acoustic"))
    (:file "_package_NED_acoustic" :depends-on ("_package"))
    (:file "NED_objects" :depends-on ("_package_NED_objects"))
    (:file "_package_NED_objects" :depends-on ("_package"))
    (:file "NED_poses" :depends-on ("_package_NED_poses"))
    (:file "_package_NED_poses" :depends-on ("_package"))
    (:file "control_efforts" :depends-on ("_package_control_efforts"))
    (:file "_package_control_efforts" :depends-on ("_package"))
    (:file "propulsion_system" :depends-on ("_package_propulsion_system"))
    (:file "_package_propulsion_system" :depends-on ("_package"))
    (:file "state" :depends-on ("_package_state"))
    (:file "_package_state" :depends-on ("_package"))
    (:file "usv_pose" :depends-on ("_package_usv_pose"))
    (:file "_package_usv_pose" :depends-on ("_package"))
  ))