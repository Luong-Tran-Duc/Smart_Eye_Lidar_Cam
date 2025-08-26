
(cl:in-package :asdf)

(defsystem "common-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ScanMapStatus" :depends-on ("_package_ScanMapStatus"))
    (:file "_package_ScanMapStatus" :depends-on ("_package"))
  ))