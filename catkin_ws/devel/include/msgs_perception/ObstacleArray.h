// Generated by gencpp from file msgs_perception/ObstacleArray.msg
// DO NOT EDIT!


#ifndef MSGS_PERCEPTION_MESSAGE_OBSTACLEARRAY_H
#define MSGS_PERCEPTION_MESSAGE_OBSTACLEARRAY_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <msgs_perception/Obstacle.h>

namespace msgs_perception
{
template <class ContainerAllocator>
struct ObstacleArray_
{
  typedef ObstacleArray_<ContainerAllocator> Type;

  ObstacleArray_()
    : header()
    , obstacle()  {
    }
  ObstacleArray_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , obstacle(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::msgs_perception::Obstacle_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::msgs_perception::Obstacle_<ContainerAllocator> >> _obstacle_type;
  _obstacle_type obstacle;





  typedef boost::shared_ptr< ::msgs_perception::ObstacleArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msgs_perception::ObstacleArray_<ContainerAllocator> const> ConstPtr;

}; // struct ObstacleArray_

typedef ::msgs_perception::ObstacleArray_<std::allocator<void> > ObstacleArray;

typedef boost::shared_ptr< ::msgs_perception::ObstacleArray > ObstacleArrayPtr;
typedef boost::shared_ptr< ::msgs_perception::ObstacleArray const> ObstacleArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msgs_perception::ObstacleArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msgs_perception::ObstacleArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::msgs_perception::ObstacleArray_<ContainerAllocator1> & lhs, const ::msgs_perception::ObstacleArray_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.obstacle == rhs.obstacle;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::msgs_perception::ObstacleArray_<ContainerAllocator1> & lhs, const ::msgs_perception::ObstacleArray_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace msgs_perception

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::msgs_perception::ObstacleArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msgs_perception::ObstacleArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msgs_perception::ObstacleArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msgs_perception::ObstacleArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msgs_perception::ObstacleArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msgs_perception::ObstacleArray_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msgs_perception::ObstacleArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f42dc2f490652df3c5d675ac2d304dc0";
  }

  static const char* value(const ::msgs_perception::ObstacleArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf42dc2f490652df3ULL;
  static const uint64_t static_value2 = 0xc5d675ac2d304dc0ULL;
};

template<class ContainerAllocator>
struct DataType< ::msgs_perception::ObstacleArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msgs_perception/ObstacleArray";
  }

  static const char* value(const ::msgs_perception::ObstacleArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msgs_perception::ObstacleArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"\n"
"msgs_perception/Obstacle[] obstacle\n"
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
"MSG: msgs_perception/Obstacle\n"
"# header for time/frame information\n"
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

  static const char* value(const ::msgs_perception::ObstacleArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msgs_perception::ObstacleArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.obstacle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ObstacleArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msgs_perception::ObstacleArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msgs_perception::ObstacleArray_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "obstacle[]" << std::endl;
    for (size_t i = 0; i < v.obstacle.size(); ++i)
    {
      s << indent << "  obstacle[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::msgs_perception::Obstacle_<ContainerAllocator> >::stream(s, indent + "    ", v.obstacle[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSGS_PERCEPTION_MESSAGE_OBSTACLEARRAY_H
