;; Auto-generated. Do not edit!


(when (boundp 'beginner_tutorials::Ambiente)
  (if (not (find-package "BEGINNER_TUTORIALS"))
    (make-package "BEGINNER_TUTORIALS"))
  (shadow 'Ambiente (find-package "BEGINNER_TUTORIALS")))
(unless (find-package "BEGINNER_TUTORIALS::AMBIENTE")
  (make-package "BEGINNER_TUTORIALS::AMBIENTE"))

(in-package "ROS")
;;//! \htmlinclude Ambiente.msg.html


(defclass beginner_tutorials::Ambiente
  :super ros::object
  :slots (_temperatura _umidade ))

(defmethod beginner_tutorials::Ambiente
  (:init
   (&key
    ((:temperatura __temperatura) 0.0)
    ((:umidade __umidade) 0.0)
    )
   (send-super :init)
   (setq _temperatura (float __temperatura))
   (setq _umidade (float __umidade))
   self)
  (:temperatura
   (&optional __temperatura)
   (if __temperatura (setq _temperatura __temperatura)) _temperatura)
  (:umidade
   (&optional __umidade)
   (if __umidade (setq _umidade __umidade)) _umidade)
  (:serialization-length
   ()
   (+
    ;; float32 _temperatura
    4
    ;; float32 _umidade
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _temperatura
       (sys::poke _temperatura (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _umidade
       (sys::poke _umidade (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _temperatura
     (setq _temperatura (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _umidade
     (setq _umidade (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get beginner_tutorials::Ambiente :md5sum-) "34ba80ca03e34a8cd5a9afb761234652")
(setf (get beginner_tutorials::Ambiente :datatype-) "beginner_tutorials/Ambiente")
(setf (get beginner_tutorials::Ambiente :definition-)
      "float32 temperatura
float32 umidade


")



(provide :beginner_tutorials/Ambiente "34ba80ca03e34a8cd5a9afb761234652")


