// Generated by gencpp from file msgs_navigation/TrajectoryError.msg
// DO NOT EDIT!


#ifndef MSGS_NAVIGATION_MESSAGE_TRAJECTORYERROR_H
#define MSGS_NAVIGATION_MESSAGE_TRAJECTORYERROR_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace msgs_navigation
{
template <class ContainerAllocator>
struct TrajectoryError_
{
  typedef TrajectoryError_<ContainerAllocator> Type;

  TrajectoryError_()
    : header()
    , enable()
    , lateral_error(0.0)
    , angular_error(0.0)
    , longitudinal_error(0.0)
    , kappa_error(0.0)  {
      enable.assign(false);
  }
  TrajectoryError_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , enable()
    , lateral_error(0.0)
    , angular_error(0.0)
    , longitudinal_error(0.0)
    , kappa_error(0.0)  {
  (void)_alloc;
      enable.assign(false);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef boost::array<uint8_t, 4>  _enable_type;
  _enable_type enable;

   typedef double _lateral_error_type;
  _lateral_error_type lateral_error;

   typedef double _angular_error_type;
  _angular_error_type angular_error;

   typedef double _longitudinal_error_type;
  _longitudinal_error_type longitudinal_error;

   typedef double _kappa_error_type;
  _kappa_error_type kappa_error;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(LATERAL)
  #undef LATERAL
#endif
#if defined(_WIN32) && defined(ANGULAR)
  #undef ANGULAR
#endif
#if defined(_WIN32) && defined(LONGITUDINAL)
  #undef LONGITUDINAL
#endif
#if defined(_WIN32) && defined(KAPPA)
  #undef KAPPA
#endif

  enum {
    LATERAL = 0u,
    ANGULAR = 1u,
    LONGITUDINAL = 2u,
    KAPPA = 3u,
  };


  typedef boost::shared_ptr< ::msgs_navigation::TrajectoryError_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msgs_navigation::TrajectoryError_<ContainerAllocator> const> ConstPtr;

}; // struct TrajectoryError_

typedef ::msgs_navigation::TrajectoryError_<std::allocator<void> > TrajectoryError;

typedef boost::shared_ptr< ::msgs_navigation::TrajectoryError > TrajectoryErrorPtr;
typedef boost::shared_ptr< ::msgs_navigation::TrajectoryError const> TrajectoryErrorConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msgs_navigation::TrajectoryError_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msgs_navigation::TrajectoryError_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::msgs_navigation::TrajectoryError_<ContainerAllocator1> & lhs, const ::msgs_navigation::TrajectoryError_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.enable == rhs.enable &&
    lhs.lateral_error == rhs.lateral_error &&
    lhs.angular_error == rhs.angular_error &&
    lhs.longitudinal_error == rhs.longitudinal_error &&
    lhs.kappa_error == rhs.kappa_error;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::msgs_navigation::TrajectoryError_<ContainerAllocator1> & lhs, const ::msgs_navigation::TrajectoryError_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace msgs_navigation

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::msgs_navigation::TrajectoryError_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msgs_navigation::TrajectoryError_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msgs_navigation::TrajectoryError_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msgs_navigation::TrajectoryError_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msgs_navigation::TrajectoryError_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msgs_navigation::TrajectoryError_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msgs_navigation::TrajectoryError_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4965b313ef7d8cf00b86bfaf40b54c20";
  }

  static const char* value(const ::msgs_navigation::TrajectoryError_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4965b313ef7d8cf0ULL;
  static const uint64_t static_value2 = 0x0b86bfaf40b54c20ULL;
};

template<class ContainerAllocator>
struct DataType< ::msgs_navigation::TrajectoryError_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msgs_navigation/TrajectoryError";
  }

  static const char* value(const ::msgs_navigation::TrajectoryError_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msgs_navigation::TrajectoryError_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"\n"
"bool[4] enable\n"
"uint8 LATERAL = 0\n"
"uint8 ANGULAR = 1\n"
"uint8 LONGITUDINAL = 2\n"
"uint8 KAPPA = 3\n"
"\n"
"float64 lateral_error\n"
"float64 angular_error\n"
"float64 longitudinal_error\n"
"float64 kappa_error\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::msgs_navigation::TrajectoryError_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msgs_navigation::TrajectoryError_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.enable);
      stream.next(m.lateral_error);
      stream.next(m.angular_error);
      stream.next(m.longitudinal_error);
      stream.next(m.kappa_error);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TrajectoryError_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msgs_navigation::TrajectoryError_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msgs_navigation::TrajectoryError_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "enable[]" << std::endl;
    for (size_t i = 0; i < v.enable.size(); ++i)
    {
      s << indent << "  enable[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.enable[i]);
    }
    s << indent << "lateral_error: ";
    Printer<double>::stream(s, indent + "  ", v.lateral_error);
    s << indent << "angular_error: ";
    Printer<double>::stream(s, indent + "  ", v.angular_error);
    s << indent << "longitudinal_error: ";
    Printer<double>::stream(s, indent + "  ", v.longitudinal_error);
    s << indent << "kappa_error: ";
    Printer<double>::stream(s, indent + "  ", v.kappa_error);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSGS_NAVIGATION_MESSAGE_TRAJECTORYERROR_H
