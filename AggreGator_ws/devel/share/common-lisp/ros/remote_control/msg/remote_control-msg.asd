
(cl:in-package :asdf)

(defsystem "remote_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "WheelMotor" :depends-on ("_package_WheelMotor"))
    (:file "_package_WheelMotor" :depends-on ("_package"))
  ))