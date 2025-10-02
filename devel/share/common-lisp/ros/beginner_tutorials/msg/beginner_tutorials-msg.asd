
(cl:in-package :asdf)

(defsystem "beginner_tutorials-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Ambiente" :depends-on ("_package_Ambiente"))
    (:file "_package_Ambiente" :depends-on ("_package"))
    (:file "Funcionarios" :depends-on ("_package_Funcionarios"))
    (:file "_package_Funcionarios" :depends-on ("_package"))
  ))