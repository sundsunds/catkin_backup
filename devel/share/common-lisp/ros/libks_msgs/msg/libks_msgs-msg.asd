
(cl:in-package :asdf)

(defsystem "libks_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SharedImageSet" :depends-on ("_package_SharedImageSet"))
    (:file "_package_SharedImageSet" :depends-on ("_package"))
    (:file "ImageSet" :depends-on ("_package_ImageSet"))
    (:file "_package_ImageSet" :depends-on ("_package"))
    (:file "MultiCameraImage" :depends-on ("_package_MultiCameraImage"))
    (:file "_package_MultiCameraImage" :depends-on ("_package"))
  ))