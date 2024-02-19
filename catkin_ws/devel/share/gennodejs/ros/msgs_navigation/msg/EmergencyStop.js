// Auto-generated. Do not edit!

// (in-package msgs_navigation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let msgs_perception = _finder('msgs_perception');

//-----------------------------------------------------------

class EmergencyStop {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.dist_to_stop = null;
      this.obstacle = null;
      this.stop_now = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('dist_to_stop')) {
        this.dist_to_stop = initObj.dist_to_stop
      }
      else {
        this.dist_to_stop = 0.0;
      }
      if (initObj.hasOwnProperty('obstacle')) {
        this.obstacle = initObj.obstacle
      }
      else {
        this.obstacle = new msgs_perception.msg.Obstacle();
      }
      if (initObj.hasOwnProperty('stop_now')) {
        this.stop_now = initObj.stop_now
      }
      else {
        this.stop_now = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EmergencyStop
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [dist_to_stop]
    bufferOffset = _serializer.float64(obj.dist_to_stop, buffer, bufferOffset);
    // Serialize message field [obstacle]
    bufferOffset = msgs_perception.msg.Obstacle.serialize(obj.obstacle, buffer, bufferOffset);
    // Serialize message field [stop_now]
    bufferOffset = _serializer.bool(obj.stop_now, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EmergencyStop
    let len;
    let data = new EmergencyStop(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [dist_to_stop]
    data.dist_to_stop = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [obstacle]
    data.obstacle = msgs_perception.msg.Obstacle.deserialize(buffer, bufferOffset);
    // Deserialize message field [stop_now]
    data.stop_now = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += msgs_perception.msg.Obstacle.getMessageSize(object.obstacle);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'msgs_navigation/EmergencyStop';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6c63fac0d11cdcd3fb7f902e34941777';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    float64 dist_to_stop
    msgs_perception/Obstacle obstacle
    bool stop_now #true to stop at the moment the behavior received the message
    
    
    
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
    MSG: msgs_perception/Obstacle
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
    const resolved = new EmergencyStop(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.dist_to_stop !== undefined) {
      resolved.dist_to_stop = msg.dist_to_stop;
    }
    else {
      resolved.dist_to_stop = 0.0
    }

    if (msg.obstacle !== undefined) {
      resolved.obstacle = msgs_perception.msg.Obstacle.Resolve(msg.obstacle)
    }
    else {
      resolved.obstacle = new msgs_perception.msg.Obstacle()
    }

    if (msg.stop_now !== undefined) {
      resolved.stop_now = msg.stop_now;
    }
    else {
      resolved.stop_now = false
    }

    return resolved;
    }
};

module.exports = EmergencyStop;
