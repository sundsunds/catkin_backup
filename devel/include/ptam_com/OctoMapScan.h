// Generated by gencpp from file ptam_com/OctoMapScan.msg
// DO NOT EDIT!


#ifndef PTAM_COM_MESSAGE_OCTOMAPSCAN_H
#define PTAM_COM_MESSAGE_OCTOMAPSCAN_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <ptam_com/OctoMapPointArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace ptam_com
{
template <class ContainerAllocator>
struct OctoMapScan_
{
  typedef OctoMapScan_<ContainerAllocator> Type;

  OctoMapScan_()
    : mapPoints()
    , keyFramePose()  {
    }
  OctoMapScan_(const ContainerAllocator& _alloc)
    : mapPoints(_alloc)
    , keyFramePose(_alloc)  {
  (void)_alloc;
    }



   typedef  ::ptam_com::OctoMapPointArray_<ContainerAllocator>  _mapPoints_type;
  _mapPoints_type mapPoints;

   typedef  ::geometry_msgs::PoseWithCovarianceStamped_<ContainerAllocator>  _keyFramePose_type;
  _keyFramePose_type keyFramePose;




  typedef boost::shared_ptr< ::ptam_com::OctoMapScan_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ptam_com::OctoMapScan_<ContainerAllocator> const> ConstPtr;

}; // struct OctoMapScan_

typedef ::ptam_com::OctoMapScan_<std::allocator<void> > OctoMapScan;

typedef boost::shared_ptr< ::ptam_com::OctoMapScan > OctoMapScanPtr;
typedef boost::shared_ptr< ::ptam_com::OctoMapScan const> OctoMapScanConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ptam_com::OctoMapScan_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ptam_com::OctoMapScan_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ptam_com

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'ptam_com': ['/home/sun/catkin_ws/src/ethzasl_ptam/ptam_com/msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ptam_com::OctoMapScan_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ptam_com::OctoMapScan_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ptam_com::OctoMapScan_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ptam_com::OctoMapScan_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ptam_com::OctoMapScan_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ptam_com::OctoMapScan_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ptam_com::OctoMapScan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6300a17df88639b05a1a89ffdf26ee86";
  }

  static const char* value(const ::ptam_com::OctoMapScan_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6300a17df88639b0ULL;
  static const uint64_t static_value2 = 0x5a1a89ffdf26ee86ULL;
};

template<class ContainerAllocator>
struct DataType< ::ptam_com::OctoMapScan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ptam_com/OctoMapScan";
  }

  static const char* value(const ::ptam_com::OctoMapScan_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ptam_com::OctoMapScan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "OctoMapPointArray mapPoints\n\
geometry_msgs/PoseWithCovarianceStamped keyFramePose\n\
\n\
\n\
================================================================================\n\
MSG: ptam_com/OctoMapPointArray\n\
OctoMapPointStamped[] mapPoints\n\
\n\
================================================================================\n\
MSG: ptam_com/OctoMapPointStamped\n\
Header header\n\
uint8 INSERT = 0\n\
uint8 UPDATE = 1\n\
uint8 DELETE = 2\n\
\n\
uint8 action\n\
geometry_msgs/Vector3 position\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: geometry_msgs/PoseWithCovarianceStamped\n\
# This expresses an estimated pose with a reference coordinate frame and timestamp\n\
\n\
Header header\n\
PoseWithCovariance pose\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseWithCovariance\n\
# This represents a pose in free space with uncertainty.\n\
\n\
Pose pose\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::ptam_com::OctoMapScan_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ptam_com::OctoMapScan_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mapPoints);
      stream.next(m.keyFramePose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct OctoMapScan_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ptam_com::OctoMapScan_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ptam_com::OctoMapScan_<ContainerAllocator>& v)
  {
    s << indent << "mapPoints: ";
    s << std::endl;
    Printer< ::ptam_com::OctoMapPointArray_<ContainerAllocator> >::stream(s, indent + "  ", v.mapPoints);
    s << indent << "keyFramePose: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseWithCovarianceStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.keyFramePose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PTAM_COM_MESSAGE_OCTOMAPSCAN_H
