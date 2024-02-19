// Generated by gencpp from file msgs_perception/Obstacle.msg
// DO NOT EDIT!


#ifndef MSGS_PERCEPTION_MESSAGE_OBSTACLE_H
#define MSGS_PERCEPTION_MESSAGE_OBSTACLE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <msgs_perception/BoundingBox.h>
#include <std_msgs/ColorRGBA.h>

namespace msgs_perception
{
template <class ContainerAllocator>
struct Obstacle_
{
  typedef Obstacle_<ContainerAllocator> Type;

  Obstacle_()
    : header()
    , id(0)
    , ns()
    , pose()
    , twist()
    , linear_acceleration()
    , scale()
    , point_cloud()
    , class_id(0)
    , classes()
    , bbox()
    , oncoming(false)
    , lat_rate(0.0)
    , track_status(0)
    , bridge_object(false)
    , color()
    , lifetime()
    , frame_locked(false)
    , animation_speed(0.0)
    , action(0)
    , type(0)
    , vehicle_model()  {
    }
  Obstacle_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , id(0)
    , ns(_alloc)
    , pose(_alloc)
    , twist(_alloc)
    , linear_acceleration(_alloc)
    , scale(_alloc)
    , point_cloud(_alloc)
    , class_id(0)
    , classes(_alloc)
    , bbox(_alloc)
    , oncoming(false)
    , lat_rate(0.0)
    , track_status(0)
    , bridge_object(false)
    , color(_alloc)
    , lifetime()
    , frame_locked(false)
    , animation_speed(0.0)
    , action(0)
    , type(0)
    , vehicle_model(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _id_type;
  _id_type id;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _ns_type;
  _ns_type ns;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _twist_type;
  _twist_type twist;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _linear_acceleration_type;
  _linear_acceleration_type linear_acceleration;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _scale_type;
  _scale_type scale;

   typedef  ::sensor_msgs::PointCloud2_<ContainerAllocator>  _point_cloud_type;
  _point_cloud_type point_cloud;

   typedef int32_t _class_id_type;
  _class_id_type class_id;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> _classes_type;
  _classes_type classes;

   typedef  ::msgs_perception::BoundingBox_<ContainerAllocator>  _bbox_type;
  _bbox_type bbox;

   typedef uint8_t _oncoming_type;
  _oncoming_type oncoming;

   typedef double _lat_rate_type;
  _lat_rate_type lat_rate;

   typedef int32_t _track_status_type;
  _track_status_type track_status;

   typedef uint8_t _bridge_object_type;
  _bridge_object_type bridge_object;

   typedef  ::std_msgs::ColorRGBA_<ContainerAllocator>  _color_type;
  _color_type color;

   typedef ros::Duration _lifetime_type;
  _lifetime_type lifetime;

   typedef uint8_t _frame_locked_type;
  _frame_locked_type frame_locked;

   typedef float _animation_speed_type;
  _animation_speed_type animation_speed;

   typedef int32_t _action_type;
  _action_type action;

   typedef int32_t _type_type;
  _type_type type;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _vehicle_model_type;
  _vehicle_model_type vehicle_model;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(ADD)
  #undef ADD
#endif
#if defined(_WIN32) && defined(MODIFY)
  #undef MODIFY
#endif
#if defined(_WIN32) && defined(DELETE)
  #undef DELETE
#endif

  enum {
    ADD = 0u,
    MODIFY = 0u,
    DELETE = 2u,
  };


  typedef boost::shared_ptr< ::msgs_perception::Obstacle_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msgs_perception::Obstacle_<ContainerAllocator> const> ConstPtr;

}; // struct Obstacle_

typedef ::msgs_perception::Obstacle_<std::allocator<void> > Obstacle;

typedef boost::shared_ptr< ::msgs_perception::Obstacle > ObstaclePtr;
typedef boost::shared_ptr< ::msgs_perception::Obstacle const> ObstacleConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msgs_perception::Obstacle_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msgs_perception::Obstacle_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::msgs_perception::Obstacle_<ContainerAllocator1> & lhs, const ::msgs_perception::Obstacle_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.id == rhs.id &&
    lhs.ns == rhs.ns &&
    lhs.pose == rhs.pose &&
    lhs.twist == rhs.twist &&
    lhs.linear_acceleration == rhs.linear_acceleration &&
    lhs.scale == rhs.scale &&
    lhs.point_cloud == rhs.point_cloud &&
    lhs.class_id == rhs.class_id &&
    lhs.classes == rhs.classes &&
    lhs.bbox == rhs.bbox &&
    lhs.oncoming == rhs.oncoming &&
    lhs.lat_rate == rhs.lat_rate &&
    lhs.track_status == rhs.track_status &&
    lhs.bridge_object == rhs.bridge_object &&
    lhs.color == rhs.color &&
    lhs.lifetime == rhs.lifetime &&
    lhs.frame_locked == rhs.frame_locked &&
    lhs.animation_speed == rhs.animation_speed &&
    lhs.action == rhs.action &&
    lhs.type == rhs.type &&
    lhs.vehicle_model == rhs.vehicle_model;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::msgs_perception::Obstacle_<ContainerAllocator1> & lhs, const ::msgs_perception::Obstacle_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace msgs_perception

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::msgs_perception::Obstacle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msgs_perception::Obstacle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msgs_perception::Obstacle_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msgs_perception::Obstacle_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msgs_perception::Obstacle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msgs_perception::Obstacle_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msgs_perception::Obstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9dd38fb85c319c66ba1631ab9f3d5a09";
  }

  static const char* value(const ::msgs_perception::Obstacle_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9dd38fb85c319c66ULL;
  static const uint64_t static_value2 = 0xba1631ab9f3d5a09ULL;
};

template<class ContainerAllocator>
struct DataType< ::msgs_perception::Obstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msgs_perception/Obstacle";
  }

  static const char* value(const ::msgs_perception::Obstacle_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msgs_perception::Obstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# header for time/frame information\n"
"Header header\n"
"\n"
"#object position \n"
"int32   id                                # object ID useful in conjunction with the namespace for manipulating and deleting the object later\n"
"string  ns                    		  # Namespace to place this object in... used in conjunction with id to create a unique name for the object\n"
"geometry_msgs/Pose    pose    		  # object pose and orientation\n"
"geometry_msgs/Twist   twist  		  # object velocity\n"
"geometry_msgs/Vector3 linear_acceleration # object acceleration\n"
"geometry_msgs/Vector3 scale       	  # Scale of the object 1,1,1 means (usually 1 meter square)\n"
"\n"
"#Fusion extras\n"
"sensor_msgs/PointCloud2 point_cloud\n"
"int32 class_id\n"
"string[] classes\n"
"msgs_perception/BoundingBox bbox\n"
"\n"
"#Radar extras\n"
"bool    oncoming\n"
"float64 lat_rate\n"
"int32   track_status         # 0 = no target, 1 = new target, 2 = new updated target, 3 = updated target, 4 = coasted target, 5 = merged target, 6 = invalid coasted target, 7 = new coasted target\n"
"bool    bridge_object        # connects two or more objects that are associated with the same obstacle\n"
"\n"
"#visualization extras\n"
"std_msgs/ColorRGBA color          # Color [0.0-1.0]\n"
"duration lifetime                 # How long the object should last before being automatically deleted.  0 means forever\n"
"bool     frame_locked             # If this marker should be frame-locked, i.e. retransformed into its frame every timestep\n"
"float32  animation_speed          # Speed of animation,\n"
"int32    action 	          # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects\n"
"uint8    ADD=0\n"
"uint8    MODIFY=0\n"
"uint8    DELETE=2\n"
"int32 type                 		      # object type : -1 unknown\n"
"string vehicle_model\n"
"\n"
"# based on Marker, See http://www.ros.org/wiki/rviz/DisplayTypes/Marker\n"
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
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Twist\n"
"# This expresses velocity in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: sensor_msgs/PointCloud2\n"
"# This message holds a collection of N-dimensional points, which may\n"
"# contain additional information such as normals, intensity, etc. The\n"
"# point data is stored as a binary blob, its layout described by the\n"
"# contents of the \"fields\" array.\n"
"\n"
"# The point cloud data may be organized 2d (image-like) or 1d\n"
"# (unordered). Point clouds organized as 2d images may be produced by\n"
"# camera depth sensors such as stereo or time-of-flight.\n"
"\n"
"# Time of sensor data acquisition, and the coordinate frame ID (for 3d\n"
"# points).\n"
"Header header\n"
"\n"
"# 2D structure of the point cloud. If the cloud is unordered, height is\n"
"# 1 and width is the length of the point cloud.\n"
"uint32 height\n"
"uint32 width\n"
"\n"
"# Describes the channels and their layout in the binary data blob.\n"
"PointField[] fields\n"
"\n"
"bool    is_bigendian # Is this data bigendian?\n"
"uint32  point_step   # Length of a point in bytes\n"
"uint32  row_step     # Length of a row in bytes\n"
"uint8[] data         # Actual point data, size is (row_step*height)\n"
"\n"
"bool is_dense        # True if there are no invalid points\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/PointField\n"
"# This message holds the description of one point entry in the\n"
"# PointCloud2 message format.\n"
"uint8 INT8    = 1\n"
"uint8 UINT8   = 2\n"
"uint8 INT16   = 3\n"
"uint8 UINT16  = 4\n"
"uint8 INT32   = 5\n"
"uint8 UINT32  = 6\n"
"uint8 FLOAT32 = 7\n"
"uint8 FLOAT64 = 8\n"
"\n"
"string name      # Name of field\n"
"uint32 offset    # Offset from start of point struct\n"
"uint8  datatype  # Datatype enumeration, see above\n"
"uint32 count     # How many elements in the field\n"
"\n"
"================================================================================\n"
"MSG: msgs_perception/BoundingBox\n"
"std_msgs/String classe\n"
"\n"
"geometry_msgs/Point p1\n"
"geometry_msgs/Point p2\n"
"geometry_msgs/Point p3\n"
"geometry_msgs/Point p4\n"
"\n"
"float64 probability\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/String\n"
"string data\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/ColorRGBA\n"
"float32 r\n"
"float32 g\n"
"float32 b\n"
"float32 a\n"
;
  }

  static const char* value(const ::msgs_perception::Obstacle_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msgs_perception::Obstacle_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.id);
      stream.next(m.ns);
      stream.next(m.pose);
      stream.next(m.twist);
      stream.next(m.linear_acceleration);
      stream.next(m.scale);
      stream.next(m.point_cloud);
      stream.next(m.class_id);
      stream.next(m.classes);
      stream.next(m.bbox);
      stream.next(m.oncoming);
      stream.next(m.lat_rate);
      stream.next(m.track_status);
      stream.next(m.bridge_object);
      stream.next(m.color);
      stream.next(m.lifetime);
      stream.next(m.frame_locked);
      stream.next(m.animation_speed);
      stream.next(m.action);
      stream.next(m.type);
      stream.next(m.vehicle_model);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Obstacle_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msgs_perception::Obstacle_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msgs_perception::Obstacle_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "ns: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.ns);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "twist: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.twist);
    s << indent << "linear_acceleration: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.linear_acceleration);
    s << indent << "scale: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.scale);
    s << indent << "point_cloud: ";
    s << std::endl;
    Printer< ::sensor_msgs::PointCloud2_<ContainerAllocator> >::stream(s, indent + "  ", v.point_cloud);
    s << indent << "class_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.class_id);
    s << indent << "classes[]" << std::endl;
    for (size_t i = 0; i < v.classes.size(); ++i)
    {
      s << indent << "  classes[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.classes[i]);
    }
    s << indent << "bbox: ";
    s << std::endl;
    Printer< ::msgs_perception::BoundingBox_<ContainerAllocator> >::stream(s, indent + "  ", v.bbox);
    s << indent << "oncoming: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.oncoming);
    s << indent << "lat_rate: ";
    Printer<double>::stream(s, indent + "  ", v.lat_rate);
    s << indent << "track_status: ";
    Printer<int32_t>::stream(s, indent + "  ", v.track_status);
    s << indent << "bridge_object: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.bridge_object);
    s << indent << "color: ";
    s << std::endl;
    Printer< ::std_msgs::ColorRGBA_<ContainerAllocator> >::stream(s, indent + "  ", v.color);
    s << indent << "lifetime: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.lifetime);
    s << indent << "frame_locked: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.frame_locked);
    s << indent << "animation_speed: ";
    Printer<float>::stream(s, indent + "  ", v.animation_speed);
    s << indent << "action: ";
    Printer<int32_t>::stream(s, indent + "  ", v.action);
    s << indent << "type: ";
    Printer<int32_t>::stream(s, indent + "  ", v.type);
    s << indent << "vehicle_model: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.vehicle_model);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSGS_PERCEPTION_MESSAGE_OBSTACLE_H