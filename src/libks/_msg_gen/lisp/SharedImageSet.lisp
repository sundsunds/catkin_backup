; Auto-generated. Do not edit!


(cl:in-package libks-msg)


;//! \htmlinclude SharedImageSet.msg.html

(cl:defclass <SharedImageSet> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (imagePtrs
    :reader imagePtrs
    :initarg :imagePtrs
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass SharedImageSet (<SharedImageSet>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SharedImageSet>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SharedImageSet)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name libks-msg:<SharedImageSet> is deprecated: use libks-msg:SharedImageSet instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SharedImageSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader libks-msg:header-val is deprecated.  Use libks-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'imagePtrs-val :lambda-list '(m))
(cl:defmethod imagePtrs-val ((m <SharedImageSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader libks-msg:imagePtrs-val is deprecated.  Use libks-msg:imagePtrs instead.")
  (imagePtrs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SharedImageSet>) ostream)
  "Serializes a message object of type '<SharedImageSet>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'imagePtrs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) ele) ostream))
   (cl:slot-value msg 'imagePtrs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SharedImageSet>) istream)
  "Deserializes a message object of type '<SharedImageSet>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'imagePtrs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'imagePtrs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SharedImageSet>)))
  "Returns string type for a message object of type '<SharedImageSet>"
  "libks/SharedImageSet")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SharedImageSet)))
  "Returns string type for a message object of type 'SharedImageSet"
  "libks/SharedImageSet")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SharedImageSet>)))
  "Returns md5sum for a message object of type '<SharedImageSet>"
  "0d17d1086d8c500eecddadc40f6db542")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SharedImageSet)))
  "Returns md5sum for a message object of type 'SharedImageSet"
  "0d17d1086d8c500eecddadc40f6db542")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SharedImageSet>)))
  "Returns full string definition for message of type '<SharedImageSet>"
  (cl:format cl:nil "Header header~%~%uint64[] imagePtrs~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SharedImageSet)))
  "Returns full string definition for message of type 'SharedImageSet"
  (cl:format cl:nil "Header header~%~%uint64[] imagePtrs~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SharedImageSet>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'imagePtrs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SharedImageSet>))
  "Converts a ROS message object to a list"
  (cl:list 'SharedImageSet
    (cl:cons ':header (header msg))
    (cl:cons ':imagePtrs (imagePtrs msg))
))
