;; Auto-generated. Do not edit!


(when (boundp 'beginner_tutorials::Funcionarios)
  (if (not (find-package "BEGINNER_TUTORIALS"))
    (make-package "BEGINNER_TUTORIALS"))
  (shadow 'Funcionarios (find-package "BEGINNER_TUTORIALS")))
(unless (find-package "BEGINNER_TUTORIALS::FUNCIONARIOS")
  (make-package "BEGINNER_TUTORIALS::FUNCIONARIOS"))

(in-package "ROS")
;;//! \htmlinclude Funcionarios.msg.html


(defclass beginner_tutorials::Funcionarios
  :super ros::object
  :slots (_nome _idade _cargo _altura ))

(defmethod beginner_tutorials::Funcionarios
  (:init
   (&key
    ((:nome __nome) "")
    ((:idade __idade) 0)
    ((:cargo __cargo) "")
    ((:altura __altura) 0.0)
    )
   (send-super :init)
   (setq _nome (string __nome))
   (setq _idade (round __idade))
   (setq _cargo (string __cargo))
   (setq _altura (float __altura))
   self)
  (:nome
   (&optional __nome)
   (if __nome (setq _nome __nome)) _nome)
  (:idade
   (&optional __idade)
   (if __idade (setq _idade __idade)) _idade)
  (:cargo
   (&optional __cargo)
   (if __cargo (setq _cargo __cargo)) _cargo)
  (:altura
   (&optional __altura)
   (if __altura (setq _altura __altura)) _altura)
  (:serialization-length
   ()
   (+
    ;; string _nome
    4 (length _nome)
    ;; int8 _idade
    1
    ;; string _cargo
    4 (length _cargo)
    ;; float64 _altura
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _nome
       (write-long (length _nome) s) (princ _nome s)
     ;; int8 _idade
       (write-byte _idade s)
     ;; string _cargo
       (write-long (length _cargo) s) (princ _cargo s)
     ;; float64 _altura
       (sys::poke _altura (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _nome
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _nome (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _idade
     (setq _idade (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _idade 127) (setq _idade (- _idade 256)))
   ;; string _cargo
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cargo (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _altura
     (setq _altura (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get beginner_tutorials::Funcionarios :md5sum-) "f599a6c815b91d516a23de2962c57921")
(setf (get beginner_tutorials::Funcionarios :datatype-) "beginner_tutorials/Funcionarios")
(setf (get beginner_tutorials::Funcionarios :definition-)
      "string nome
int8 idade
string cargo
float64 altura

")



(provide :beginner_tutorials/Funcionarios "f599a6c815b91d516a23de2962c57921")


