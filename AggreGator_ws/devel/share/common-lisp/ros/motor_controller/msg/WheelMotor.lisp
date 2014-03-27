; Auto-generated. Do not edit!


(cl:in-package motor_controller-msg)


;//! \htmlinclude WheelMotor.msg.html

(cl:defclass <WheelMotor> (roslisp-msg-protocol:ros-message)
  ((LF_motorVal
    :reader LF_motorVal
    :initarg :LF_motorVal
    :type cl:fixnum
    :initform 0)
   (RF_motorVal
    :reader RF_motorVal
    :initarg :RF_motorVal
    :type cl:fixnum
    :initform 0)
   (RR_motorVal
    :reader RR_motorVal
    :initarg :RR_motorVal
    :type cl:fixnum
    :initform 0)
   (LR_motorVal
    :reader LR_motorVal
    :initarg :LR_motorVal
    :type cl:fixnum
    :initform 0))
)

(cl:defclass WheelMotor (<WheelMotor>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WheelMotor>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WheelMotor)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motor_controller-msg:<WheelMotor> is deprecated: use motor_controller-msg:WheelMotor instead.")))

(cl:ensure-generic-function 'LF_motorVal-val :lambda-list '(m))
(cl:defmethod LF_motorVal-val ((m <WheelMotor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_controller-msg:LF_motorVal-val is deprecated.  Use motor_controller-msg:LF_motorVal instead.")
  (LF_motorVal m))

(cl:ensure-generic-function 'RF_motorVal-val :lambda-list '(m))
(cl:defmethod RF_motorVal-val ((m <WheelMotor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_controller-msg:RF_motorVal-val is deprecated.  Use motor_controller-msg:RF_motorVal instead.")
  (RF_motorVal m))

(cl:ensure-generic-function 'RR_motorVal-val :lambda-list '(m))
(cl:defmethod RR_motorVal-val ((m <WheelMotor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_controller-msg:RR_motorVal-val is deprecated.  Use motor_controller-msg:RR_motorVal instead.")
  (RR_motorVal m))

(cl:ensure-generic-function 'LR_motorVal-val :lambda-list '(m))
(cl:defmethod LR_motorVal-val ((m <WheelMotor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_controller-msg:LR_motorVal-val is deprecated.  Use motor_controller-msg:LR_motorVal instead.")
  (LR_motorVal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WheelMotor>) ostream)
  "Serializes a message object of type '<WheelMotor>"
  (cl:let* ((signed (cl:slot-value msg 'LF_motorVal)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'RF_motorVal)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'RR_motorVal)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'LR_motorVal)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WheelMotor>) istream)
  "Deserializes a message object of type '<WheelMotor>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'LF_motorVal) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'RF_motorVal) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'RR_motorVal) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'LR_motorVal) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WheelMotor>)))
  "Returns string type for a message object of type '<WheelMotor>"
  "motor_controller/WheelMotor")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WheelMotor)))
  "Returns string type for a message object of type 'WheelMotor"
  "motor_controller/WheelMotor")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WheelMotor>)))
  "Returns md5sum for a message object of type '<WheelMotor>"
  "f0f17242b728bc2f9b92df609716f415")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WheelMotor)))
  "Returns md5sum for a message object of type 'WheelMotor"
  "f0f17242b728bc2f9b92df609716f415")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WheelMotor>)))
  "Returns full string definition for message of type '<WheelMotor>"
  (cl:format cl:nil "~%int16 LF_motorVal~%int16 RF_motorVal~%int16 RR_motorVal~%int16 LR_motorVal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WheelMotor)))
  "Returns full string definition for message of type 'WheelMotor"
  (cl:format cl:nil "~%int16 LF_motorVal~%int16 RF_motorVal~%int16 RR_motorVal~%int16 LR_motorVal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WheelMotor>))
  (cl:+ 0
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WheelMotor>))
  "Converts a ROS message object to a list"
  (cl:list 'WheelMotor
    (cl:cons ':LF_motorVal (LF_motorVal msg))
    (cl:cons ':RF_motorVal (RF_motorVal msg))
    (cl:cons ':RR_motorVal (RR_motorVal msg))
    (cl:cons ':LR_motorVal (LR_motorVal msg))
))
