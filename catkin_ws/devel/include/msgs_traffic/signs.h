// Generated by gencpp from file msgs_traffic/signs.msg
// DO NOT EDIT!


#ifndef MSGS_TRAFFIC_MESSAGE_SIGNS_H
#define MSGS_TRAFFIC_MESSAGE_SIGNS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace msgs_traffic
{
template <class ContainerAllocator>
struct signs_
{
  typedef signs_<ContainerAllocator> Type;

  signs_()
    {
    }
  signs_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }





// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(UNKNOWN)
  #undef UNKNOWN
#endif
#if defined(_WIN32) && defined(STOP)
  #undef STOP
#endif
#if defined(_WIN32) && defined(SPEED_LIMIT)
  #undef SPEED_LIMIT
#endif
#if defined(_WIN32) && defined(PEDESTRIAN_CROSSING)
  #undef PEDESTRIAN_CROSSING
#endif
#if defined(_WIN32) && defined(SPEED_BUMP)
  #undef SPEED_BUMP
#endif
#if defined(_WIN32) && defined(TRAFFIC_LIGHT)
  #undef TRAFFIC_LIGHT
#endif

  enum {
    UNKNOWN = 0u,
    STOP = 1u,
    SPEED_LIMIT = 2u,
    PEDESTRIAN_CROSSING = 3u,
    SPEED_BUMP = 4u,
    TRAFFIC_LIGHT = 5u,
  };


  typedef boost::shared_ptr< ::msgs_traffic::signs_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msgs_traffic::signs_<ContainerAllocator> const> ConstPtr;

}; // struct signs_

typedef ::msgs_traffic::signs_<std::allocator<void> > signs;

typedef boost::shared_ptr< ::msgs_traffic::signs > signsPtr;
typedef boost::shared_ptr< ::msgs_traffic::signs const> signsConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msgs_traffic::signs_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msgs_traffic::signs_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace msgs_traffic

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::msgs_traffic::signs_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msgs_traffic::signs_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msgs_traffic::signs_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msgs_traffic::signs_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msgs_traffic::signs_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msgs_traffic::signs_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msgs_traffic::signs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e583dc6ff2378238d65c0f560909a609";
  }

  static const char* value(const ::msgs_traffic::signs_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe583dc6ff2378238ULL;
  static const uint64_t static_value2 = 0xd65c0f560909a609ULL;
};

template<class ContainerAllocator>
struct DataType< ::msgs_traffic::signs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msgs_traffic/signs";
  }

  static const char* value(const ::msgs_traffic::signs_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msgs_traffic::signs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#list of all traffic signs\n"
"\n"
"uint8 UNKNOWN             = 0\n"
"uint8 STOP                = 1\n"
"uint8 SPEED_LIMIT         = 2\n"
"uint8 PEDESTRIAN_CROSSING = 3\n"
"uint8 SPEED_BUMP          = 4\n"
"uint8 TRAFFIC_LIGHT       = 5\n"
;
  }

  static const char* value(const ::msgs_traffic::signs_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msgs_traffic::signs_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct signs_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msgs_traffic::signs_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::msgs_traffic::signs_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // MSGS_TRAFFIC_MESSAGE_SIGNS_H
