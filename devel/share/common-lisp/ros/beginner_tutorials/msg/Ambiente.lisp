; Auto-generated. Do not edit!


(cl:in-package beginner_tutorials-msg)


;//! \htmlinclude Ambiente.msg.html

(cl:defclass <Ambiente> (roslisp-msg-protocol:ros-message)
  ((temperatura
    :reader temperatura
    :initarg :temperatura
    :type cl:float
    :initform 0.0)
   (umidade
    :reader umidade
    :initarg :umidade
    :type cl:float
    :initform 0.0))
)

(cl:defclass Ambiente (<Ambiente>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Ambiente>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Ambiente)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-msg:<Ambiente> is deprecated: use beginner_tutorials-msg:Ambiente instead.")))

(cl:ensure-generic-function 'temperatura-val :lambda-list '(m))
(cl:defmethod temperatura-val ((m <Ambiente>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:temperatura-val is deprecated.  Use beginner_tutorials-msg:temperatura instead.")
  (temperatura m))

(cl:ensure-generic-function 'umidade-val :lambda-list '(m))
(cl:defmethod umidade-val ((m <Ambiente>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:umidade-val is deprecated.  Use beginner_tutorials-msg:umidade instead.")
  (umidade m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Ambiente>) ostream)
  "Serializes a message object of type '<Ambiente>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temperatura))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'umidade))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Ambiente>) istream)
  "Deserializes a message object of type '<Ambiente>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperatura) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'umidade) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Ambiente>)))
  "Returns string type for a message object of type '<Ambiente>"
  "beginner_tutorials/Ambiente")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Ambiente)))
  "Returns string type for a message object of type 'Ambiente"
  "beginner_tutorials/Ambiente")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Ambiente>)))
  "Returns md5sum for a message object of type '<Ambiente>"
  "34ba80ca03e34a8cd5a9afb761234652")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Ambiente)))
  "Returns md5sum for a message object of type 'Ambiente"
  "34ba80ca03e34a8cd5a9afb761234652")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Ambiente>)))
  "Returns full string definition for message of type '<Ambiente>"
  (cl:format cl:nil "float32 temperatura~%float32 umidade~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Ambiente)))
  "Returns full string definition for message of type 'Ambiente"
  (cl:format cl:nil "float32 temperatura~%float32 umidade~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Ambiente>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Ambiente>))
  "Converts a ROS message object to a list"
  (cl:list 'Ambiente
    (cl:cons ':temperatura (temperatura msg))
    (cl:cons ':umidade (umidade msg))
))
