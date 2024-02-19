// Generated by gencpp from file msgs_navigation/SpeedConstraint.msg
// DO NOT EDIT!


#ifndef MSGS_NAVIGATION_MESSAGE_SPEEDCONSTRAINT_H
#define MSGS_NAVIGATION_MESSAGE_SPEEDCONSTRAINT_H


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
struct SpeedConstraint_
{
  typedef SpeedConstraint_<ContainerAllocator> Type;

  SpeedConstraint_()
    : header()
    , speed(0.0)
    , decrease_rate(0.0)
    , reason()  {
    }
  SpeedConstraint_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , speed(0.0)
    , decrease_rate(0.0)
    , reason(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _speed_type;
  _speed_type speed;

   typedef double _decrease_rate_type;
  _decrease_rate_type decrease_rate;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _reason_type;
  _reason_type reason;





  typedef boost::shared_ptr< ::msgs_navigation::SpeedConstraint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msgs_navigation::SpeedConstraint_<ContainerAllocator> const> ConstPtr;

}; // struct SpeedConstraint_

typedef ::msgs_navigation::SpeedConstraint_<std::allocator<void> > SpeedConstraint;

typedef boost::shared_ptr< ::msgs_navigation::SpeedConstraint > SpeedConstraintPtr;
typedef boost::shared_ptr< ::msgs_navigation::SpeedConstraint const> SpeedConstraintConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msgs_navigation::SpeedConstraint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msgs_navigation::SpeedConstraint_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::msgs_navigation::SpeedConstraint_<ContainerAllocator1> & lhs, const ::msgs_navigation::SpeedConstraint_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.speed == rhs.speed &&
    lhs.decrease_rate == rhs.decrease_rate &&
    lhs.reason == rhs.reason;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::msgs_navigation::SpeedConstraint_<ContainerAllocator1> & lhs, const ::msgs_navigation::SpeedConstraint_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace msgs_navigation

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::msgs_navigation::SpeedConstraint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msgs_navigation::SpeedConstraint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msgs_navigation::SpeedConstraint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msgs_navigation::SpeedConstraint_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msgs_navigation::SpeedConstraint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msgs_navigation::SpeedConstraint_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msgs_navigation::SpeedConstraint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "09819746c374e0d4aba5a41ddeab09cb";
  }

  static const char* value(const ::msgs_navigation::SpeedConstraint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x09819746c374e0d4ULL;
  static const uint64_t static_value2 = 0xaba5a41ddeab09cbULL;
};

template<class ContainerAllocator>
struct DataType< ::msgs_navigation::SpeedConstraint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msgs_navigation/SpeedConstraint";
  }

  static const char* value(const ::msgs_navigation::SpeedConstraint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msgs_navigation::SpeedConstraint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"float64 speed\n"
"float64 decrease_rate\n"
"string reason\n"
"\n"
"\n"
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

  static const char* value(const ::msgs_navigation::SpeedConstraint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msgs_navigation::SpeedConstraint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.speed);
      stream.next(m.decrease_rate);
      stream.next(m.reason);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SpeedConstraint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msgs_navigation::SpeedConstraint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msgs_navigation::SpeedConstraint_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "speed: ";
    Printer<double>::stream(s, indent + "  ", v.speed);
    s << indent << "decrease_rate: ";
    Printer<double>::stream(s, indent + "  ", v.decrease_rate);
    s << indent << "reason: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.reason);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSGS_NAVIGATION_MESSAGE_SPEEDCONSTRAINT_H
