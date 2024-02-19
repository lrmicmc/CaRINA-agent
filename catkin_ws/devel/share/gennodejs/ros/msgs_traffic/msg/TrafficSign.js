// Auto-generated. Do not edit!

// (in-package msgs_traffic.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let signs = require('./signs.js');
let traffic_light = require('./traffic_light.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class TrafficSign {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.type = null;
      this.name = null;
      this.list = null;
      this.value = null;
      this.traffic_light = null;
      this.pose = null;
      this.length = null;
      this.road_id = null;
      this.lanes = null;
    }
    else {
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('list')) {
        this.list = initObj.list
      }
      else {
        this.list = new signs();
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0.0;
      }
      if (initObj.hasOwnProperty('traffic_light')) {
        this.traffic_light = initObj.traffic_light
      }
      else {
        this.traffic_light = new traffic_light();
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('length')) {
        this.length = initObj.length
      }
      else {
        this.length = 0.0;
      }
      if (initObj.hasOwnProperty('road_id')) {
        this.road_id = initObj.road_id
      }
      else {
        this.road_id = 0;
      }
      if (initObj.hasOwnProperty('lanes')) {
        this.lanes = initObj.lanes
      }
      else {
        this.lanes = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrafficSign
    // Serialize message field [type]
    bufferOffset = _serializer.uint8(obj.type, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [list]
    bufferOffset = signs.serialize(obj.list, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.float64(obj.value, buffer, bufferOffset);
    // Serialize message field [traffic_light]
    bufferOffset = traffic_light.serialize(obj.traffic_light, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [length]
    bufferOffset = _serializer.float64(obj.length, buffer, bufferOffset);
    // Serialize message field [road_id]
    bufferOffset = _serializer.uint32(obj.road_id, buffer, bufferOffset);
    // Serialize message field [lanes]
    bufferOffset = _arraySerializer.uint32(obj.lanes, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrafficSign
    let len;
    let data = new TrafficSign(null);
    // Deserialize message field [type]
    data.type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [list]
    data.list = signs.deserialize(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [traffic_light]
    data.traffic_light = traffic_light.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [length]
    data.length = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [road_id]
    data.road_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [lanes]
    data.lanes = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.pose);
    length += 4 * object.lanes.length;
    return length + 30;
  }

  static datatype() {
    // Returns string type for a message object
    return 'msgs_traffic/TrafficSign';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a4364772549aa27d16150f86259ed928';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrafficSign(null);
    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.list !== undefined) {
      resolved.list = signs.Resolve(msg.list)
    }
    else {
      resolved.list = new signs()
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0.0
    }

    if (msg.traffic_light !== undefined) {
      resolved.traffic_light = traffic_light.Resolve(msg.traffic_light)
    }
    else {
      resolved.traffic_light = new traffic_light()
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.PoseStamped.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.length !== undefined) {
      resolved.length = msg.length;
    }
    else {
      resolved.length = 0.0
    }

    if (msg.road_id !== undefined) {
      resolved.road_id = msg.road_id;
    }
    else {
      resolved.road_id = 0
    }

    if (msg.lanes !== undefined) {
      resolved.lanes = msg.lanes;
    }
    else {
      resolved.lanes = []
    }

    return resolved;
    }
};

module.exports = TrafficSign;
