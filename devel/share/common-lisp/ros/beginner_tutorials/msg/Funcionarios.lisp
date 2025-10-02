; Auto-generated. Do not edit!


(cl:in-package beginner_tutorials-msg)


;//! \htmlinclude Funcionarios.msg.html

(cl:defclass <Funcionarios> (roslisp-msg-protocol:ros-message)
  ((nome
    :reader nome
    :initarg :nome
    :type cl:string
    :initform "")
   (idade
    :reader idade
    :initarg :idade
    :type cl:fixnum
    :initform 0)
   (cargo
    :reader cargo
    :initarg :cargo
    :type cl:string
    :initform "")
   (altura
    :reader altura
    :initarg :altura
    :type cl:float
    :initform 0.0))
)

(cl:defclass Funcionarios (<Funcionarios>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Funcionarios>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Funcionarios)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-msg:<Funcionarios> is deprecated: use beginner_tutorials-msg:Funcionarios instead.")))

(cl:ensure-generic-function 'nome-val :lambda-list '(m))
(cl:defmethod nome-val ((m <Funcionarios>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:nome-val is deprecated.  Use beginner_tutorials-msg:nome instead.")
  (nome m))

(cl:ensure-generic-function 'idade-val :lambda-list '(m))
(cl:defmethod idade-val ((m <Funcionarios>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:idade-val is deprecated.  Use beginner_tutorials-msg:idade instead.")
  (idade m))

(cl:ensure-generic-function 'cargo-val :lambda-list '(m))
(cl:defmethod cargo-val ((m <Funcionarios>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:cargo-val is deprecated.  Use beginner_tutorials-msg:cargo instead.")
  (cargo m))

(cl:ensure-generic-function 'altura-val :lambda-list '(m))
(cl:defmethod altura-val ((m <Funcionarios>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:altura-val is deprecated.  Use beginner_tutorials-msg:altura instead.")
  (altura m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Funcionarios>) ostream)
  "Serializes a message object of type '<Funcionarios>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'nome))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'nome))
  (cl:let* ((signed (cl:slot-value msg 'idade)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cargo))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cargo))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'altura))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Funcionarios>) istream)
  "Deserializes a message object of type '<Funcionarios>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nome) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'nome) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'idade) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cargo) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cargo) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'altura) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Funcionarios>)))
  "Returns string type for a message object of type '<Funcionarios>"
  "beginner_tutorials/Funcionarios")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Funcionarios)))
  "Returns string type for a message object of type 'Funcionarios"
  "beginner_tutorials/Funcionarios")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Funcionarios>)))
  "Returns md5sum for a message object of type '<Funcionarios>"
  "f599a6c815b91d516a23de2962c57921")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Funcionarios)))
  "Returns md5sum for a message object of type 'Funcionarios"
  "f599a6c815b91d516a23de2962c57921")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Funcionarios>)))
  "Returns full string definition for message of type '<Funcionarios>"
  (cl:format cl:nil "string nome~%int8 idade~%string cargo~%float64 altura~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Funcionarios)))
  "Returns full string definition for message of type 'Funcionarios"
  (cl:format cl:nil "string nome~%int8 idade~%string cargo~%float64 altura~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Funcionarios>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'nome))
     1
     4 (cl:length (cl:slot-value msg 'cargo))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Funcionarios>))
  "Converts a ROS message object to a list"
  (cl:list 'Funcionarios
    (cl:cons ':nome (nome msg))
    (cl:cons ':idade (idade msg))
    (cl:cons ':cargo (cargo msg))
    (cl:cons ':altura (altura msg))
))
