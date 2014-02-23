
(cl:in-package :asdf)

(defsystem "motor_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "I2CMSG" :depends-on ("_package_I2CMSG"))
    (:file "_package_I2CMSG" :depends-on ("_package"))
    (:file "motorMSG" :depends-on ("_package_motorMSG"))
    (:file "_package_motorMSG" :depends-on ("_package"))
  ))