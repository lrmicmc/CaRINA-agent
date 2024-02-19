// Generated by gencpp from file msgs_navigation/Path.msg
// DO NOT EDIT!


#ifndef MSGS_NAVIGATION_MESSAGE_PATH_H
#define MSGS_NAVIGATION_MESSAGE_PATH_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <msgs_navigation/TrajectoryPoint.h>

namespace msgs_navigation
{
template <class ContainerAllocator>
struct Path_
{
  typedef Path_<ContainerAllocator> Type;

  Path_()
    : header()
    , path()
    , size(0)  {
    }
  Path_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , path(_alloc)
    , size(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::msgs_navigation::TrajectoryPoint_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::msgs_navigation::TrajectoryPoint_<ContainerAllocator> >> _path_type;
  _path_type path;

   typedef int32_t _size_type;
  _size_type size;





  typedef boost::shared_ptr< ::msgs_navigation::Path_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msgs_navigation::Path_<ContainerAllocator> const> ConstPtr;

}; // struct Path_

typedef ::msgs_navigation::Path_<std::allocator<void> > Path;

typedef boost::shared_ptr< ::msgs_navigation::Path > PathPtr;
typedef boost::shared_ptr< ::msgs_navigation::Path const> PathConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msgs_navigation::Path_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msgs_navigation::Path_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::msgs_navigation::Path_<ContainerAllocator1> & lhs, const ::msgs_navigation::Path_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.path == rhs.path &&
    lhs.size == rhs.size;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::msgs_navigation::Path_<ContainerAllocator1> & lhs, const ::msgs_navigation::Path_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace msgs_navigation

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::msgs_navigation::Path_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msgs_navigation::Path_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msgs_navigation::Path_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msgs_navigation::Path_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msgs_navigation::Path_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msgs_navigation::Path_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msgs_navigation::Path_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9cb18433fe776145fb64e8e568e28b11";
  }

  static const char* value(const ::msgs_navigation::Path_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9cb18433fe776145ULL;
  static const uint64_t static_value2 = 0xfb64e8e568e28b11ULL;
};

template<class ContainerAllocator>
struct DataType< ::msgs_navigation::Path_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msgs_navigation/Path";
  }

  static const char* value(const ::msgs_navigation::Path_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msgs_navigation::Path_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"msgs_navigation/TrajectoryPoint[] path\n"
"int32 size\n"
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
"\n"
"================================================================================\n"
"MSG: msgs_navigation/TrajectoryPoint\n"
"# Clothoid data\n"
"\n"
"float64[] point\n"
"\n"
"uint8 X = 0\n"
"uint8 Y = 1\n"
"uint8 KAPPA = 2\n"
"uint8 ANGLE = 3\n"
"uint8 LENGTH = 4\n"
"uint8 SPEED = 5\n"
"uint8 KAPPA_DERIVATIVE = 6\n"
"\n"
"float64 point_number\n"
"bool end_track\n"
;
  }

  static const char* value(const ::msgs_navigation::Path_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msgs_navigation::Path_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.path);
      stream.next(m.size);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Path_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msgs_navigation::Path_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msgs_navigation::Path_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "path[]" << std::endl;
    for (size_t i = 0; i < v.path.size(); ++i)
    {
      s << indent << "  path[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::msgs_navigation::TrajectoryPoint_<ContainerAllocator> >::stream(s, indent + "    ", v.path[i]);
    }
    s << indent << "size: ";
    Printer<int32_t>::stream(s, indent + "  ", v.size);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSGS_NAVIGATION_MESSAGE_PATH_H