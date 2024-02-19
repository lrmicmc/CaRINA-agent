// Auto-generated. Do not edit!

// (in-package msgs_perception.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BoundingBox = require('./BoundingBox.js');
let geometry_msgs = _finder('geometry_msgs');
let sensor_msgs = _finder('sensor_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Obstacle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.id = null;
      this.ns = null;
      this.pose = null;
      this.twist = null;
      this.linear_acceleration = null;
      this.scale = null;
      this.point_cloud = null;
      this.class_id = null;
      this.classes = null;
      this.bbox = null;
      this.oncoming = null;
      this.lat_rate = null;
      this.track_status = null;
      this.bridge_object = null;
      this.color = null;
      this.lifetime = null;
      this.frame_locked = null;
      this.animation_speed = null;
      this.action = null;
      this.type = null;
      this.vehicle_model = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('ns')) {
        this.ns = initObj.ns
      }
      else {
        this.ns = '';
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('twist')) {
        this.twist = initObj.twist
      }
      else {
        this.twist = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('linear_acceleration')) {
        this.linear_acceleration = initObj.linear_acceleration
      }
      else {
        this.linear_acceleration = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('scale')) {
        this.scale = initObj.scale
      }
      else {
        this.scale = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('point_cloud')) {
        this.point_cloud = initObj.point_cloud
      }
      else {
        this.point_cloud = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('class_id')) {
        this.class_id = initObj.class_id
      }
      else {
        this.class_id = 0;
      }
      if (initObj.hasOwnProperty('classes')) {
        this.classes = initObj.classes
      }
      else {
        this.classes = [];
      }
      if (initObj.hasOwnProperty('bbox')) {
        this.bbox = initObj.bbox
      }
      else {
        this.bbox = new BoundingBox();
      }
      if (initObj.hasOwnProperty('oncoming')) {
        this.oncoming = initObj.oncoming
      }
      else {
        this.oncoming = false;
      }
      if (initObj.hasOwnProperty('lat_rate')) {
        this.lat_rate = initObj.lat_rate
      }
      else {
        this.lat_rate = 0.0;
      }
      if (initObj.hasOwnProperty('track_status')) {
        this.track_status = initObj.track_status
      }
      else {
        this.track_status = 0;
      }
      if (initObj.hasOwnProperty('bridge_object')) {
        this.bridge_object = initObj.bridge_object
      }
      else {
        this.bridge_object = false;
      }
      if (initObj.hasOwnProperty('color')) {
        this.color = initObj.color
      }
      else {
        this.color = new std_msgs.msg.ColorRGBA();
      }
      if (initObj.hasOwnProperty('lifetime')) {
        this.lifetime = initObj.lifetime
      }
      else {
        this.lifetime = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('frame_locked')) {
        this.frame_locked = initObj.frame_locked
      }
      else {
        this.frame_locked = false;
      }
      if (initObj.hasOwnProperty('animation_speed')) {
        this.animation_speed = initObj.animation_speed
      }
      else {
        this.animation_speed = 0.0;
      }
      if (initObj.hasOwnProperty('action')) {
        this.action = initObj.action
      }
      else {
        this.action = 0;
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('vehicle_model')) {
        this.vehicle_model = initObj.vehicle_model
      }
      else {
        this.vehicle_model = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Obstacle
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [ns]
    bufferOffset = _serializer.string(obj.ns, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [twist]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.twist, buffer, bufferOffset);
    // Serialize message field [linear_acceleration]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.linear_acceleration, buffer, bufferOffset);
    // Serialize message field [scale]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.scale, buffer, bufferOffset);
    // Serialize message field [point_cloud]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.point_cloud, buffer, bufferOffset);
    // Serialize message field [class_id]
    bufferOffset = _serializer.int32(obj.class_id, buffer, bufferOffset);
    // Serialize message field [classes]
    bufferOffset = _arraySerializer.string(obj.classes, buffer, bufferOffset, null);
    // Serialize message field [bbox]
    bufferOffset = BoundingBox.serialize(obj.bbox, buffer, bufferOffset);
    // Serialize message field [oncoming]
    bufferOffset = _serializer.bool(obj.oncoming, buffer, bufferOffset);
    // Serialize message field [lat_rate]
    bufferOffset = _serializer.float64(obj.lat_rate, buffer, bufferOffset);
    // Serialize message field [track_status]
    bufferOffset = _serializer.int32(obj.track_status, buffer, bufferOffset);
    // Serialize message field [bridge_object]
    bufferOffset = _serializer.bool(obj.bridge_object, buffer, bufferOffset);
    // Serialize message field [color]
    bufferOffset = std_msgs.msg.ColorRGBA.serialize(obj.color, buffer, bufferOffset);
    // Serialize message field [lifetime]
    bufferOffset = _serializer.duration(obj.lifetime, buffer, bufferOffset);
    // Serialize message field [frame_locked]
    bufferOffset = _serializer.bool(obj.frame_locked, buffer, bufferOffset);
    // Serialize message field [animation_speed]
    bufferOffset = _serializer.float32(obj.animation_speed, buffer, bufferOffset);
    // Serialize message field [action]
    bufferOffset = _serializer.int32(obj.action, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.int32(obj.type, buffer, bufferOffset);
    // Serialize message field [vehicle_model]
    bufferOffset = _serializer.string(obj.vehicle_model, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Obstacle
    let len;
    let data = new Obstacle(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ns]
    data.ns = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [twist]
    data.twist = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [linear_acceleration]
    data.linear_acceleration = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [scale]
    data.scale = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [point_cloud]
    data.point_cloud = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [class_id]
    data.class_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [classes]
    data.classes = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [bbox]
    data.bbox = BoundingBox.deserialize(buffer, bufferOffset);
    // Deserialize message field [oncoming]
    data.oncoming = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [lat_rate]
    data.lat_rate = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [track_status]
    data.track_status = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [bridge_object]
    data.bridge_object = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [color]
    data.color = std_msgs.msg.ColorRGBA.deserialize(buffer, bufferOffset);
    // Deserialize message field [lifetime]
    data.lifetime = _deserializer.duration(buffer, bufferOffset);
    // Deserialize message field [frame_locked]
    data.frame_locked = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [animation_speed]
    data.animation_speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [action]
    data.action = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [vehicle_model]
    data.vehicle_model = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.ns);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.point_cloud);
    object.classes.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += BoundingBox.getMessageSize(object.bbox);
    length += _getByteLength(object.vehicle_model);
    return length + 223;
  }

  static datatype() {
    // Returns string type for a message object
    return 'msgs_perception/Obstacle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9dd38fb85c319c66ba1631ab9f3d5a09';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # header for time/frame information
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Obstacle(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.ns !== undefined) {
      resolved.ns = msg.ns;
    }
    else {
      resolved.ns = ''
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.twist !== undefined) {
      resolved.twist = geometry_msgs.msg.Twist.Resolve(msg.twist)
    }
    else {
      resolved.twist = new geometry_msgs.msg.Twist()
    }

    if (msg.linear_acceleration !== undefined) {
      resolved.linear_acceleration = geometry_msgs.msg.Vector3.Resolve(msg.linear_acceleration)
    }
    else {
      resolved.linear_acceleration = new geometry_msgs.msg.Vector3()
    }

    if (msg.scale !== undefined) {
      resolved.scale = geometry_msgs.msg.Vector3.Resolve(msg.scale)
    }
    else {
      resolved.scale = new geometry_msgs.msg.Vector3()
    }

    if (msg.point_cloud !== undefined) {
      resolved.point_cloud = sensor_msgs.msg.PointCloud2.Resolve(msg.point_cloud)
    }
    else {
      resolved.point_cloud = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.class_id !== undefined) {
      resolved.class_id = msg.class_id;
    }
    else {
      resolved.class_id = 0
    }

    if (msg.classes !== undefined) {
      resolved.classes = msg.classes;
    }
    else {
      resolved.classes = []
    }

    if (msg.bbox !== undefined) {
      resolved.bbox = BoundingBox.Resolve(msg.bbox)
    }
    else {
      resolved.bbox = new BoundingBox()
    }

    if (msg.oncoming !== undefined) {
      resolved.oncoming = msg.oncoming;
    }
    else {
      resolved.oncoming = false
    }

    if (msg.lat_rate !== undefined) {
      resolved.lat_rate = msg.lat_rate;
    }
    else {
      resolved.lat_rate = 0.0
    }

    if (msg.track_status !== undefined) {
      resolved.track_status = msg.track_status;
    }
    else {
      resolved.track_status = 0
    }

    if (msg.bridge_object !== undefined) {
      resolved.bridge_object = msg.bridge_object;
    }
    else {
      resolved.bridge_object = false
    }

    if (msg.color !== undefined) {
      resolved.color = std_msgs.msg.ColorRGBA.Resolve(msg.color)
    }
    else {
      resolved.color = new std_msgs.msg.ColorRGBA()
    }

    if (msg.lifetime !== undefined) {
      resolved.lifetime = msg.lifetime;
    }
    else {
      resolved.lifetime = {secs: 0, nsecs: 0}
    }

    if (msg.frame_locked !== undefined) {
      resolved.frame_locked = msg.frame_locked;
    }
    else {
      resolved.frame_locked = false
    }

    if (msg.animation_speed !== undefined) {
      resolved.animation_speed = msg.animation_speed;
    }
    else {
      resolved.animation_speed = 0.0
    }

    if (msg.action !== undefined) {
      resolved.action = msg.action;
    }
    else {
      resolved.action = 0
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.vehicle_model !== undefined) {
      resolved.vehicle_model = msg.vehicle_model;
    }
    else {
      resolved.vehicle_model = ''
    }

    return resolved;
    }
};

// Constants for message
Obstacle.Constants = {
  ADD: 0,
  MODIFY: 0,
  DELETE: 2,
}

module.exports = Obstacle;
