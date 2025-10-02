;; Auto-generated. Do not edit!


(when (boundp 'beginner_tutorials::ControleIluminacao)
  (if (not (find-package "BEGINNER_TUTORIALS"))
    (make-package "BEGINNER_TUTORIALS"))
  (shadow 'ControleIluminacao (find-package "BEGINNER_TUTORIALS")))
(unless (find-package "BEGINNER_TUTORIALS::CONTROLEILUMINACAO")
  (make-package "BEGINNER_TUTORIALS::CONTROLEILUMINACAO"))
(unless (find-package "BEGINNER_TUTORIALS::CONTROLEILUMINACAOREQUEST")
  (make-package "BEGINNER_TUTORIALS::CONTROLEILUMINACAOREQUEST"))
(unless (find-package "BEGINNER_TUTORIALS::CONTROLEILUMINACAORESPONSE")
  (make-package "BEGINNER_TUTORIALS::CONTROLEILUMINACAORESPONSE"))

(in-package "ROS")





(defclass beginner_tutorials::ControleIluminacaoRequest
  :super ros::object
  :slots (_comando ))

(defmethod beginner_tutorials::ControleIluminacaoRequest
  (:init
   (&key
    ((:comando __comando) "")
    )
   (send-super :init)
   (setq _comando (string __comando))
   self)
  (:comando
   (&optional __comando)
   (if __comando (setq _comando __comando)) _comando)
  (:serialization-length
   ()
   (+
    ;; string _comando
    4 (length _comando)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _comando
       (write-long (length _comando) s) (princ _comando s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _comando
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _comando (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass beginner_tutorials::ControleIluminacaoResponse
  :super ros::object
  :slots (_resposta ))

(defmethod beginner_tutorials::ControleIluminacaoResponse
  (:init
   (&key
    ((:resposta __resposta) "")
    )
   (send-super :init)
   (setq _resposta (string __resposta))
   self)
  (:resposta
   (&optional __resposta)
   (if __resposta (setq _resposta __resposta)) _resposta)
  (:serialization-length
   ()
   (+
    ;; string _resposta
    4 (length _resposta)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _resposta
       (write-long (length _resposta) s) (princ _resposta s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _resposta
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _resposta (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass beginner_tutorials::ControleIluminacao
  :super ros::object
  :slots ())

(setf (get beginner_tutorials::ControleIluminacao :md5sum-) "dd563be63c4578244cd65e069bc1911e")
(setf (get beginner_tutorials::ControleIluminacao :datatype-) "beginner_tutorials/ControleIluminacao")
(setf (get beginner_tutorials::ControleIluminacao :request) beginner_tutorials::ControleIluminacaoRequest)
(setf (get beginner_tutorials::ControleIluminacao :response) beginner_tutorials::ControleIluminacaoResponse)

(defmethod beginner_tutorials::ControleIluminacaoRequest
  (:response () (instance beginner_tutorials::ControleIluminacaoResponse :init)))

(setf (get beginner_tutorials::ControleIluminacaoRequest :md5sum-) "dd563be63c4578244cd65e069bc1911e")
(setf (get beginner_tutorials::ControleIluminacaoRequest :datatype-) "beginner_tutorials/ControleIluminacaoRequest")
(setf (get beginner_tutorials::ControleIluminacaoRequest :definition-)
      "string comando
---
string resposta

")

(setf (get beginner_tutorials::ControleIluminacaoResponse :md5sum-) "dd563be63c4578244cd65e069bc1911e")
(setf (get beginner_tutorials::ControleIluminacaoResponse :datatype-) "beginner_tutorials/ControleIluminacaoResponse")
(setf (get beginner_tutorials::ControleIluminacaoResponse :definition-)
      "string comando
---
string resposta

")



(provide :beginner_tutorials/ControleIluminacao "dd563be63c4578244cd65e069bc1911e")


