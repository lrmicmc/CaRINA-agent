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

//-----------------------------------------------------------

class TrajectoryError {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.enable = null;
      this.lateral_error = null;
      this.angular_error = null;
      this.longitudinal_error = null;
      this.kappa_error = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('enable')) {
        this.enable = initObj.enable
      }
      else {
        this.enable = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('lateral_error')) {
        this.lateral_error = initObj.lateral_error
      }
      else {
        this.lateral_error = 0.0;
      }
      if (initObj.hasOwnProperty('angular_error')) {
        this.angular_error = initObj.angular_error
      }
      else {
        this.angular_error = 0.0;
      }
      if (initObj.hasOwnProperty('longitudinal_error')) {
        this.longitudinal_error = initObj.longitudinal_error
      }
      else {
        this.longitudinal_error = 0.0;
      }
      if (initObj.hasOwnProperty('kappa_error')) {
        this.kappa_error = initObj.kappa_error
      }
      else {
        this.kappa_error = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryError
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [enable] has the right length
    if (obj.enable.length !== 4) {
      throw new Error('Unable to serialize array field enable - length must be 4')
    }
    // Serialize message field [enable]
    bufferOffset = _arraySerializer.bool(obj.enable, buffer, bufferOffset, 4);
    // Serialize message field [lateral_error]
    bufferOffset = _serializer.float64(obj.lateral_error, buffer, bufferOffset);
    // Serialize message field [angular_error]
    bufferOffset = _serializer.float64(obj.angular_error, buffer, bufferOffset);
    // Serialize message field [longitudinal_error]
    bufferOffset = _serializer.float64(obj.longitudinal_error, buffer, bufferOffset);
    // Serialize message field [kappa_error]
    bufferOffset = _serializer.float64(obj.kappa_error, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryError
    let len;
    let data = new TrajectoryError(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [enable]
    data.enable = _arrayDeserializer.bool(buffer, bufferOffset, 4)
    // Deserialize message field [lateral_error]
    data.lateral_error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [angular_error]
    data.angular_error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [longitudinal_error]
    data.longitudinal_error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [kappa_error]
    data.kappa_error = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'msgs_navigation/TrajectoryError';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4965b313ef7d8cf00b86bfaf40b54c20';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    bool[4] enable
    uint8 LATERAL = 0
    uint8 ANGULAR = 1
    uint8 LONGITUDINAL = 2
    uint8 KAPPA = 3
    
    float64 lateral_error
    float64 angular_error
    float64 longitudinal_error
    float64 kappa_error
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajectoryError(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.enable !== undefined) {
      resolved.enable = msg.enable;
    }
    else {
      resolved.enable = new Array(4).fill(0)
    }

    if (msg.lateral_error !== undefined) {
      resolved.lateral_error = msg.lateral_error;
    }
    else {
      resolved.lateral_error = 0.0
    }

    if (msg.angular_error !== undefined) {
      resolved.angular_error = msg.angular_error;
    }
    else {
      resolved.angular_error = 0.0
    }

    if (msg.longitudinal_error !== undefined) {
      resolved.longitudinal_error = msg.longitudinal_error;
    }
    else {
      resolved.longitudinal_error = 0.0
    }

    if (msg.kappa_error !== undefined) {
      resolved.kappa_error = msg.kappa_error;
    }
    else {
      resolved.kappa_error = 0.0
    }

    return resolved;
    }
};

// Constants for message
TrajectoryError.Constants = {
  LATERAL: 0,
  ANGULAR: 1,
  LONGITUDINAL: 2,
  KAPPA: 3,
}

module.exports = TrajectoryError;
