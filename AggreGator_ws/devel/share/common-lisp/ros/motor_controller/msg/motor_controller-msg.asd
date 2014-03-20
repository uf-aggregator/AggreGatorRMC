
(cl:in-package :asdf)

(defsystem "motor_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "I2CGeneric" :depends-on ("_package_I2CGeneric"))
    (:file "_package_I2CGeneric" :depends-on ("_package"))
    (:file "WheelMotor" :depends-on ("_package_WheelMotor"))
    (:file "_package_WheelMotor" :depends-on ("_package"))
  ))