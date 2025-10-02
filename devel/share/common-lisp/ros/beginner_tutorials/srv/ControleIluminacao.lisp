; Auto-generated. Do not edit!


(cl:in-package beginner_tutorials-srv)


;//! \htmlinclude ControleIluminacao-request.msg.html

(cl:defclass <ControleIluminacao-request> (roslisp-msg-protocol:ros-message)
  ((comando
    :reader comando
    :initarg :comando
    :type cl:string
    :initform ""))
)

(cl:defclass ControleIluminacao-request (<ControleIluminacao-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControleIluminacao-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControleIluminacao-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-srv:<ControleIluminacao-request> is deprecated: use beginner_tutorials-srv:ControleIluminacao-request instead.")))

(cl:ensure-generic-function 'comando-val :lambda-list '(m))
(cl:defmethod comando-val ((m <ControleIluminacao-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-srv:comando-val is deprecated.  Use beginner_tutorials-srv:comando instead.")
  (comando m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControleIluminacao-request>) ostream)
  "Serializes a message object of type '<ControleIluminacao-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'comando))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'comando))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControleIluminacao-request>) istream)
  "Deserializes a message object of type '<ControleIluminacao-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'comando) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'comando) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControleIluminacao-request>)))
  "Returns string type for a service object of type '<ControleIluminacao-request>"
  "beginner_tutorials/ControleIluminacaoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControleIluminacao-request)))
  "Returns string type for a service object of type 'ControleIluminacao-request"
  "beginner_tutorials/ControleIluminacaoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControleIluminacao-request>)))
  "Returns md5sum for a message object of type '<ControleIluminacao-request>"
  "dd563be63c4578244cd65e069bc1911e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControleIluminacao-request)))
  "Returns md5sum for a message object of type 'ControleIluminacao-request"
  "dd563be63c4578244cd65e069bc1911e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControleIluminacao-request>)))
  "Returns full string definition for message of type '<ControleIluminacao-request>"
  (cl:format cl:nil "string comando~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControleIluminacao-request)))
  "Returns full string definition for message of type 'ControleIluminacao-request"
  (cl:format cl:nil "string comando~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControleIluminacao-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'comando))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControleIluminacao-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ControleIluminacao-request
    (cl:cons ':comando (comando msg))
))
;//! \htmlinclude ControleIluminacao-response.msg.html

(cl:defclass <ControleIluminacao-response> (roslisp-msg-protocol:ros-message)
  ((resposta
    :reader resposta
    :initarg :resposta
    :type cl:string
    :initform ""))
)

(cl:defclass ControleIluminacao-response (<ControleIluminacao-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControleIluminacao-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControleIluminacao-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-srv:<ControleIluminacao-response> is deprecated: use beginner_tutorials-srv:ControleIluminacao-response instead.")))

(cl:ensure-generic-function 'resposta-val :lambda-list '(m))
(cl:defmethod resposta-val ((m <ControleIluminacao-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-srv:resposta-val is deprecated.  Use beginner_tutorials-srv:resposta instead.")
  (resposta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControleIluminacao-response>) ostream)
  "Serializes a message object of type '<ControleIluminacao-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'resposta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'resposta))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControleIluminacao-response>) istream)
  "Deserializes a message object of type '<ControleIluminacao-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'resposta) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'resposta) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControleIluminacao-response>)))
  "Returns string type for a service object of type '<ControleIluminacao-response>"
  "beginner_tutorials/ControleIluminacaoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControleIluminacao-response)))
  "Returns string type for a service object of type 'ControleIluminacao-response"
  "beginner_tutorials/ControleIluminacaoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControleIluminacao-response>)))
  "Returns md5sum for a message object of type '<ControleIluminacao-response>"
  "dd563be63c4578244cd65e069bc1911e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControleIluminacao-response)))
  "Returns md5sum for a message object of type 'ControleIluminacao-response"
  "dd563be63c4578244cd65e069bc1911e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControleIluminacao-response>)))
  "Returns full string definition for message of type '<ControleIluminacao-response>"
  (cl:format cl:nil "string resposta~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControleIluminacao-response)))
  "Returns full string definition for message of type 'ControleIluminacao-response"
  (cl:format cl:nil "string resposta~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControleIluminacao-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'resposta))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControleIluminacao-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ControleIluminacao-response
    (cl:cons ':resposta (resposta msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ControleIluminacao)))
  'ControleIluminacao-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ControleIluminacao)))
  'ControleIluminacao-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControleIluminacao)))
  "Returns string type for a service object of type '<ControleIluminacao>"
  "beginner_tutorials/ControleIluminacao")