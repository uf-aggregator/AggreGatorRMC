; Auto-generated. Do not edit!


(cl:in-package motor_controller-msg)


;//! \htmlinclude I2CMSG.msg.html

(cl:defclass <I2CMSG> (roslisp-msg-protocol:ros-message)
  ((addr
    :reader addr
    :initarg :addr
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type cl:fixnum
    :initform 0))
)

(cl:defclass I2CMSG (<I2CMSG>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <I2CMSG>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'I2CMSG)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motor_controller-msg:<I2CMSG> is deprecated: use motor_controller-msg:I2CMSG instead.")))

(cl:ensure-generic-function 'addr-val :lambda-list '(m))
(cl:defmethod addr-val ((m <I2CMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_controller-msg:addr-val is deprecated.  Use motor_controller-msg:addr instead.")
  (addr m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <I2CMSG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_controller-msg:data-val is deprecated.  Use motor_controller-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <I2CMSG>) ostream)
  "Serializes a message object of type '<I2CMSG>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'addr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'data)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <I2CMSG>) istream)
  "Deserializes a message object of type '<I2CMSG>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'addr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'data)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<I2CMSG>)))
  "Returns string type for a message object of type '<I2CMSG>"
  "motor_controller/I2CMSG")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'I2CMSG)))
  "Returns string type for a message object of type 'I2CMSG"
  "motor_controller/I2CMSG")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<I2CMSG>)))
  "Returns md5sum for a message object of type '<I2CMSG>"
  "179f5c810135f5d6d358345775bc956b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'I2CMSG)))
  "Returns md5sum for a message object of type 'I2CMSG"
  "179f5c810135f5d6d358345775bc956b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<I2CMSG>)))
  "Returns full string definition for message of type '<I2CMSG>"
  (cl:format cl:nil "uint8 addr~%uint16 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'I2CMSG)))
  "Returns full string definition for message of type 'I2CMSG"
  (cl:format cl:nil "uint8 addr~%uint16 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <I2CMSG>))
  (cl:+ 0
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <I2CMSG>))
  "Converts a ROS message object to a list"
  (cl:list 'I2CMSG
    (cl:cons ':addr (addr msg))
    (cl:cons ':data (data msg))
))
