# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from msgs_perception/Obstacle.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy
import geometry_msgs.msg
import msgs_perception.msg
import sensor_msgs.msg
import std_msgs.msg

class Obstacle(genpy.Message):
  _md5sum = "9dd38fb85c319c66ba1631ab9f3d5a09"
  _type = "msgs_perception/Obstacle"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """# header for time/frame information
Header header

#object position 
int32   id                                # object ID useful in conjunction with the namespace for manipulating and deleting the object later
string  ns                    		  # Namespace to place this object in... used in conjunction with id to create a unique name for the object
geometry_msgs/Pose    pose    		  # object pose and orientation
geometry_msgs/Twist   twist  		  # object velocity
geometry_msgs/Vector3 linear_acceleration # object acceleration
geometry_msgs/Vector3 scale       	  # Scale of the object 1,1,1 means (usually 1 meter square)

#Fusion extras
sensor_msgs/PointCloud2 point_cloud
int32 class_id
string[] classes
msgs_perception/BoundingBox bbox

#Radar extras
bool    oncoming
float64 lat_rate
int32   track_status         # 0 = no target, 1 = new target, 2 = new updated target, 3 = updated target, 4 = coasted target, 5 = merged target, 6 = invalid coasted target, 7 = new coasted target
bool    bridge_object        # connects two or more objects that are associated with the same obstacle

#visualization extras
std_msgs/ColorRGBA color          # Color [0.0-1.0]
duration lifetime                 # How long the object should last before being automatically deleted.  0 means forever
bool     frame_locked             # If this marker should be frame-locked, i.e. retransformed into its frame every timestep
float32  animation_speed          # Speed of animation,
int32    action 	          # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
uint8    ADD=0
uint8    MODIFY=0
uint8    DELETE=2
int32 type                 		      # object type : -1 unknown
string vehicle_model

# based on Marker, See http://www.ros.org/wiki/rviz/DisplayTypes/Marker

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

================================================================================
MSG: msgs_perception/BoundingBox
std_msgs/String classe

geometry_msgs/Point p1
geometry_msgs/Point p2
geometry_msgs/Point p3
geometry_msgs/Point p4

float64 probability

================================================================================
MSG: std_msgs/String
string data

================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
"""
  # Pseudo-constants
  ADD = 0
  MODIFY = 0
  DELETE = 2

  __slots__ = ['header','id','ns','pose','twist','linear_acceleration','scale','point_cloud','class_id','classes','bbox','oncoming','lat_rate','track_status','bridge_object','color','lifetime','frame_locked','animation_speed','action','type','vehicle_model']
  _slot_types = ['std_msgs/Header','int32','string','geometry_msgs/Pose','geometry_msgs/Twist','geometry_msgs/Vector3','geometry_msgs/Vector3','sensor_msgs/PointCloud2','int32','string[]','msgs_perception/BoundingBox','bool','float64','int32','bool','std_msgs/ColorRGBA','duration','bool','float32','int32','int32','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,id,ns,pose,twist,linear_acceleration,scale,point_cloud,class_id,classes,bbox,oncoming,lat_rate,track_status,bridge_object,color,lifetime,frame_locked,animation_speed,action,type,vehicle_model

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Obstacle, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.id is None:
        self.id = 0
      if self.ns is None:
        self.ns = ''
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
      if self.twist is None:
        self.twist = geometry_msgs.msg.Twist()
      if self.linear_acceleration is None:
        self.linear_acceleration = geometry_msgs.msg.Vector3()
      if self.scale is None:
        self.scale = geometry_msgs.msg.Vector3()
      if self.point_cloud is None:
        self.point_cloud = sensor_msgs.msg.PointCloud2()
      if self.class_id is None:
        self.class_id = 0
      if self.classes is None:
        self.classes = []
      if self.bbox is None:
        self.bbox = msgs_perception.msg.BoundingBox()
      if self.oncoming is None:
        self.oncoming = False
      if self.lat_rate is None:
        self.lat_rate = 0.
      if self.track_status is None:
        self.track_status = 0
      if self.bridge_object is None:
        self.bridge_object = False
      if self.color is None:
        self.color = std_msgs.msg.ColorRGBA()
      if self.lifetime is None:
        self.lifetime = genpy.Duration()
      if self.frame_locked is None:
        self.frame_locked = False
      if self.animation_speed is None:
        self.animation_speed = 0.
      if self.action is None:
        self.action = 0
      if self.type is None:
        self.type = 0
      if self.vehicle_model is None:
        self.vehicle_model = ''
    else:
      self.header = std_msgs.msg.Header()
      self.id = 0
      self.ns = ''
      self.pose = geometry_msgs.msg.Pose()
      self.twist = geometry_msgs.msg.Twist()
      self.linear_acceleration = geometry_msgs.msg.Vector3()
      self.scale = geometry_msgs.msg.Vector3()
      self.point_cloud = sensor_msgs.msg.PointCloud2()
      self.class_id = 0
      self.classes = []
      self.bbox = msgs_perception.msg.BoundingBox()
      self.oncoming = False
      self.lat_rate = 0.
      self.track_status = 0
      self.bridge_object = False
      self.color = std_msgs.msg.ColorRGBA()
      self.lifetime = genpy.Duration()
      self.frame_locked = False
      self.animation_speed = 0.
      self.action = 0
      self.type = 0
      self.vehicle_model = ''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.id
      buff.write(_get_struct_i().pack(_x))
      _x = self.ns
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_19d3I().pack(_x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.twist.linear.x, _x.twist.linear.y, _x.twist.linear.z, _x.twist.angular.x, _x.twist.angular.y, _x.twist.angular.z, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z, _x.scale.x, _x.scale.y, _x.scale.z, _x.point_cloud.header.seq, _x.point_cloud.header.stamp.secs, _x.point_cloud.header.stamp.nsecs))
      _x = self.point_cloud.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2I().pack(_x.point_cloud.height, _x.point_cloud.width))
      length = len(self.point_cloud.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.point_cloud.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1
        buff.write(_get_struct_IBI().pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_get_struct_B2I().pack(_x.point_cloud.is_bigendian, _x.point_cloud.point_step, _x.point_cloud.row_step))
      _x = self.point_cloud.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.Struct('<I%sB'%length).pack(length, *_x))
      else:
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_Bi().pack(_x.point_cloud.is_dense, _x.class_id))
      length = len(self.classes)
      buff.write(_struct_I.pack(length))
      for val1 in self.classes:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.Struct('<I%ss'%length).pack(length, val1))
      _x = self.bbox.classe.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_13dBdiB4f2iBf2i().pack(_x.bbox.p1.x, _x.bbox.p1.y, _x.bbox.p1.z, _x.bbox.p2.x, _x.bbox.p2.y, _x.bbox.p2.z, _x.bbox.p3.x, _x.bbox.p3.y, _x.bbox.p3.z, _x.bbox.p4.x, _x.bbox.p4.y, _x.bbox.p4.z, _x.bbox.probability, _x.oncoming, _x.lat_rate, _x.track_status, _x.bridge_object, _x.color.r, _x.color.g, _x.color.b, _x.color.a, _x.lifetime.secs, _x.lifetime.nsecs, _x.frame_locked, _x.animation_speed, _x.action, _x.type))
      _x = self.vehicle_model
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
      if self.twist is None:
        self.twist = geometry_msgs.msg.Twist()
      if self.linear_acceleration is None:
        self.linear_acceleration = geometry_msgs.msg.Vector3()
      if self.scale is None:
        self.scale = geometry_msgs.msg.Vector3()
      if self.point_cloud is None:
        self.point_cloud = sensor_msgs.msg.PointCloud2()
      if self.bbox is None:
        self.bbox = msgs_perception.msg.BoundingBox()
      if self.color is None:
        self.color = std_msgs.msg.ColorRGBA()
      if self.lifetime is None:
        self.lifetime = genpy.Duration()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (self.id,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.ns = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.ns = str[start:end]
      _x = self
      start = end
      end += 164
      (_x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.twist.linear.x, _x.twist.linear.y, _x.twist.linear.z, _x.twist.angular.x, _x.twist.angular.y, _x.twist.angular.z, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z, _x.scale.x, _x.scale.y, _x.scale.z, _x.point_cloud.header.seq, _x.point_cloud.header.stamp.secs, _x.point_cloud.header.stamp.nsecs,) = _get_struct_19d3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.point_cloud.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.point_cloud.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.point_cloud.height, _x.point_cloud.width,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.point_cloud.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _get_struct_IBI().unpack(str[start:end])
        self.point_cloud.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.point_cloud.is_bigendian, _x.point_cloud.point_step, _x.point_cloud.row_step,) = _get_struct_B2I().unpack(str[start:end])
      self.point_cloud.is_bigendian = bool(self.point_cloud.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.point_cloud.data = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.point_cloud.is_dense, _x.class_id,) = _get_struct_Bi().unpack(str[start:end])
      self.point_cloud.is_dense = bool(self.point_cloud.is_dense)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.classes = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1 = str[start:end]
        self.classes.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.bbox.classe.data = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.bbox.classe.data = str[start:end]
      _x = self
      start = end
      end += 155
      (_x.bbox.p1.x, _x.bbox.p1.y, _x.bbox.p1.z, _x.bbox.p2.x, _x.bbox.p2.y, _x.bbox.p2.z, _x.bbox.p3.x, _x.bbox.p3.y, _x.bbox.p3.z, _x.bbox.p4.x, _x.bbox.p4.y, _x.bbox.p4.z, _x.bbox.probability, _x.oncoming, _x.lat_rate, _x.track_status, _x.bridge_object, _x.color.r, _x.color.g, _x.color.b, _x.color.a, _x.lifetime.secs, _x.lifetime.nsecs, _x.frame_locked, _x.animation_speed, _x.action, _x.type,) = _get_struct_13dBdiB4f2iBf2i().unpack(str[start:end])
      self.oncoming = bool(self.oncoming)
      self.bridge_object = bool(self.bridge_object)
      self.frame_locked = bool(self.frame_locked)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.vehicle_model = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.vehicle_model = str[start:end]
      self.lifetime.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.id
      buff.write(_get_struct_i().pack(_x))
      _x = self.ns
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_19d3I().pack(_x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.twist.linear.x, _x.twist.linear.y, _x.twist.linear.z, _x.twist.angular.x, _x.twist.angular.y, _x.twist.angular.z, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z, _x.scale.x, _x.scale.y, _x.scale.z, _x.point_cloud.header.seq, _x.point_cloud.header.stamp.secs, _x.point_cloud.header.stamp.nsecs))
      _x = self.point_cloud.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2I().pack(_x.point_cloud.height, _x.point_cloud.width))
      length = len(self.point_cloud.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.point_cloud.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1
        buff.write(_get_struct_IBI().pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_get_struct_B2I().pack(_x.point_cloud.is_bigendian, _x.point_cloud.point_step, _x.point_cloud.row_step))
      _x = self.point_cloud.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.Struct('<I%sB'%length).pack(length, *_x))
      else:
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_Bi().pack(_x.point_cloud.is_dense, _x.class_id))
      length = len(self.classes)
      buff.write(_struct_I.pack(length))
      for val1 in self.classes:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.Struct('<I%ss'%length).pack(length, val1))
      _x = self.bbox.classe.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_13dBdiB4f2iBf2i().pack(_x.bbox.p1.x, _x.bbox.p1.y, _x.bbox.p1.z, _x.bbox.p2.x, _x.bbox.p2.y, _x.bbox.p2.z, _x.bbox.p3.x, _x.bbox.p3.y, _x.bbox.p3.z, _x.bbox.p4.x, _x.bbox.p4.y, _x.bbox.p4.z, _x.bbox.probability, _x.oncoming, _x.lat_rate, _x.track_status, _x.bridge_object, _x.color.r, _x.color.g, _x.color.b, _x.color.a, _x.lifetime.secs, _x.lifetime.nsecs, _x.frame_locked, _x.animation_speed, _x.action, _x.type))
      _x = self.vehicle_model
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
      if self.twist is None:
        self.twist = geometry_msgs.msg.Twist()
      if self.linear_acceleration is None:
        self.linear_acceleration = geometry_msgs.msg.Vector3()
      if self.scale is None:
        self.scale = geometry_msgs.msg.Vector3()
      if self.point_cloud is None:
        self.point_cloud = sensor_msgs.msg.PointCloud2()
      if self.bbox is None:
        self.bbox = msgs_perception.msg.BoundingBox()
      if self.color is None:
        self.color = std_msgs.msg.ColorRGBA()
      if self.lifetime is None:
        self.lifetime = genpy.Duration()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (self.id,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.ns = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.ns = str[start:end]
      _x = self
      start = end
      end += 164
      (_x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.twist.linear.x, _x.twist.linear.y, _x.twist.linear.z, _x.twist.angular.x, _x.twist.angular.y, _x.twist.angular.z, _x.linear_acceleration.x, _x.linear_acceleration.y, _x.linear_acceleration.z, _x.scale.x, _x.scale.y, _x.scale.z, _x.point_cloud.header.seq, _x.point_cloud.header.stamp.secs, _x.point_cloud.header.stamp.nsecs,) = _get_struct_19d3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.point_cloud.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.point_cloud.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.point_cloud.height, _x.point_cloud.width,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.point_cloud.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _get_struct_IBI().unpack(str[start:end])
        self.point_cloud.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.point_cloud.is_bigendian, _x.point_cloud.point_step, _x.point_cloud.row_step,) = _get_struct_B2I().unpack(str[start:end])
      self.point_cloud.is_bigendian = bool(self.point_cloud.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.point_cloud.data = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.point_cloud.is_dense, _x.class_id,) = _get_struct_Bi().unpack(str[start:end])
      self.point_cloud.is_dense = bool(self.point_cloud.is_dense)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.classes = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1 = str[start:end]
        self.classes.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.bbox.classe.data = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.bbox.classe.data = str[start:end]
      _x = self
      start = end
      end += 155
      (_x.bbox.p1.x, _x.bbox.p1.y, _x.bbox.p1.z, _x.bbox.p2.x, _x.bbox.p2.y, _x.bbox.p2.z, _x.bbox.p3.x, _x.bbox.p3.y, _x.bbox.p3.z, _x.bbox.p4.x, _x.bbox.p4.y, _x.bbox.p4.z, _x.bbox.probability, _x.oncoming, _x.lat_rate, _x.track_status, _x.bridge_object, _x.color.r, _x.color.g, _x.color.b, _x.color.a, _x.lifetime.secs, _x.lifetime.nsecs, _x.frame_locked, _x.animation_speed, _x.action, _x.type,) = _get_struct_13dBdiB4f2iBf2i().unpack(str[start:end])
      self.oncoming = bool(self.oncoming)
      self.bridge_object = bool(self.bridge_object)
      self.frame_locked = bool(self.frame_locked)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.vehicle_model = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.vehicle_model = str[start:end]
      self.lifetime.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_13dBdiB4f2iBf2i = None
def _get_struct_13dBdiB4f2iBf2i():
    global _struct_13dBdiB4f2iBf2i
    if _struct_13dBdiB4f2iBf2i is None:
        _struct_13dBdiB4f2iBf2i = struct.Struct("<13dBdiB4f2iBf2i")
    return _struct_13dBdiB4f2iBf2i
_struct_19d3I = None
def _get_struct_19d3I():
    global _struct_19d3I
    if _struct_19d3I is None:
        _struct_19d3I = struct.Struct("<19d3I")
    return _struct_19d3I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_B2I = None
def _get_struct_B2I():
    global _struct_B2I
    if _struct_B2I is None:
        _struct_B2I = struct.Struct("<B2I")
    return _struct_B2I
_struct_Bi = None
def _get_struct_Bi():
    global _struct_Bi
    if _struct_Bi is None:
        _struct_Bi = struct.Struct("<Bi")
    return _struct_Bi
_struct_IBI = None
def _get_struct_IBI():
    global _struct_IBI
    if _struct_IBI is None:
        _struct_IBI = struct.Struct("<IBI")
    return _struct_IBI
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
