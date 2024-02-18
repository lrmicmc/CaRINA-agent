// Auto-generated. Do not edit!

// (in-package msgs_traffic.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TrafficSign = require('./TrafficSign.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TrafficSignArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.signs = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('signs')) {
        this.signs = initObj.signs
      }
      else {
        this.signs = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrafficSignArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [signs]
    // Serialize the length for message field [signs]
    bufferOffset = _serializer.uint32(obj.signs.length, buffer, bufferOffset);
    obj.signs.forEach((val) => {
      bufferOffset = TrafficSign.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrafficSignArray
    let len;
    let data = new TrafficSignArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [signs]
    // Deserialize array length for message field [signs]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.signs = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.signs[i] = TrafficSign.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.signs.forEach((val) => {
      length += TrafficSign.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'msgs_traffic/TrafficSignArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '52473d5a9fb0a322cd148dfb905888a7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    msgs_traffic/TrafficSign[] signs
    
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
    MSG: msgs_traffic/TrafficSign
    #traffic sign classification
    uint8 type
    string name
    msgs_traffic/signs list
    
    #traffic sign semantic
    float64 value
    msgs_traffic/traffic_light traffic_light
    
    #traffic sign localization 
    geometry_msgs/PoseStamped pose
    float64 length
    uint32 road_id 
    uint32[] lanes
    
    
    
    
    
    
    ================================================================================
    MSG: msgs_traffic/signs
    #list of all traffic signs
    
    uint8 UNKNOWN             = 0
    uint8 STOP                = 1
    uint8 SPEED_LIMIT         = 2
    uint8 PEDESTRIAN_CROSSING = 3
    uint8 SPEED_BUMP          = 4
    uint8 TRAFFIC_LIGHT       = 5
    
    ================================================================================
    MSG: msgs_traffic/traffic_light
    uint8 RED    = 0
    uint8 GREEN  = 1
    uint8 YELLOW = 2
    
    uint8 color
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrafficSignArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.signs !== undefined) {
      resolved.signs = new Array(msg.signs.length);
      for (let i = 0; i < resolved.signs.length; ++i) {
        resolved.signs[i] = TrafficSign.Resolve(msg.signs[i]);
      }
    }
    else {
      resolved.signs = []
    }

    return resolved;
    }
};

module.exports = TrafficSignArray;
