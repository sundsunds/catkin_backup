; Auto-generated. Do not edit!


(cl:in-package libks_msgs-msg)


;//! \htmlinclude MultiCameraImage.msg.html

(cl:defclass <MultiCameraImage> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (cam0
    :reader cam0
    :initarg :cam0
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (cam1
    :reader cam1
    :initarg :cam1
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (cam2
    :reader cam2
    :initarg :cam2
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (cam3
    :reader cam3
    :initarg :cam3
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass MultiCameraImage (<MultiCameraImage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MultiCameraImage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MultiCameraImage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name libks_msgs-msg:<MultiCameraImage> is deprecated: use libks_msgs-msg:MultiCameraImage instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MultiCameraImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader libks_msgs-msg:header-val is deprecated.  Use libks_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'cam0-val :lambda-list '(m))
(cl:defmethod cam0-val ((m <MultiCameraImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader libks_msgs-msg:cam0-val is deprecated.  Use libks_msgs-msg:cam0 instead.")
  (cam0 m))

(cl:ensure-generic-function 'cam1-val :lambda-list '(m))
(cl:defmethod cam1-val ((m <MultiCameraImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader libks_msgs-msg:cam1-val is deprecated.  Use libks_msgs-msg:cam1 instead.")
  (cam1 m))

(cl:ensure-generic-function 'cam2-val :lambda-list '(m))
(cl:defmethod cam2-val ((m <MultiCameraImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader libks_msgs-msg:cam2-val is deprecated.  Use libks_msgs-msg:cam2 instead.")
  (cam2 m))

(cl:ensure-generic-function 'cam3-val :lambda-list '(m))
(cl:defmethod cam3-val ((m <MultiCameraImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader libks_msgs-msg:cam3-val is deprecated.  Use libks_msgs-msg:cam3 instead.")
  (cam3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MultiCameraImage>) ostream)
  "Serializes a message object of type '<MultiCameraImage>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cam0) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cam1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cam2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cam3) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MultiCameraImage>) istream)
  "Deserializes a message object of type '<MultiCameraImage>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cam0) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cam1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cam2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cam3) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MultiCameraImage>)))
  "Returns string type for a message object of type '<MultiCameraImage>"
  "libks_msgs/MultiCameraImage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MultiCameraImage)))
  "Returns string type for a message object of type 'MultiCameraImage"
  "libks_msgs/MultiCameraImage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MultiCameraImage>)))
  "Returns md5sum for a message object of type '<MultiCameraImage>"
  "8b44608bfb0d2204398483be4d01ac1f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MultiCameraImage)))
  "Returns md5sum for a message object of type 'MultiCameraImage"
  "8b44608bfb0d2204398483be4d01ac1f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MultiCameraImage>)))
  "Returns full string definition for message of type '<MultiCameraImage>"
  (cl:format cl:nil "# Old message type! Only used for backward compatibility!~%~%# Up to 4 cameras are supported but not all cameras~%# need to be activated.~%~%Header header~%~%sensor_msgs/Image cam0~%sensor_msgs/Image cam1~%sensor_msgs/Image cam2~%sensor_msgs/Image cam3~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MultiCameraImage)))
  "Returns full string definition for message of type 'MultiCameraImage"
  (cl:format cl:nil "# Old message type! Only used for backward compatibility!~%~%# Up to 4 cameras are supported but not all cameras~%# need to be activated.~%~%Header header~%~%sensor_msgs/Image cam0~%sensor_msgs/Image cam1~%sensor_msgs/Image cam2~%sensor_msgs/Image cam3~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MultiCameraImage>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cam0))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cam1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cam2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cam3))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MultiCameraImage>))
  "Converts a ROS message object to a list"
  (cl:list 'MultiCameraImage
    (cl:cons ':header (header msg))
    (cl:cons ':cam0 (cam0 msg))
    (cl:cons ':cam1 (cam1 msg))
    (cl:cons ':cam2 (cam2 msg))
    (cl:cons ':cam3 (cam3 msg))
))
