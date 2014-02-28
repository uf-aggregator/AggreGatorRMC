
(cl:in-package :asdf)

(defsystem "remote_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "motorMSG" :depends-on ("_package_motorMSG"))
    (:file "_package_motorMSG" :depends-on ("_package"))
  ))