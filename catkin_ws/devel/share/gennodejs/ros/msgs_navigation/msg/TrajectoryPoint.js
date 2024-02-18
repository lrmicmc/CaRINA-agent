// Auto-generated. Do not edit!

// (in-package msgs_navigation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class TrajectoryPoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.point = null;
      this.point_number = null;
      this.end_track = null;
    }
    else {
      if (initObj.hasOwnProperty('point')) {
        this.point = initObj.point
      }
      else {
        this.point = [];
      }
      if (initObj.hasOwnProperty('point_number')) {
        this.point_number = initObj.point_number
      }
      else {
        this.point_number = 0.0;
      }
      if (initObj.hasOwnProperty('end_track')) {
        this.end_track = initObj.end_track
      }
      else {
        this.end_track = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryPoint
    // Serialize message field [point]
    bufferOffset = _arraySerializer.float64(obj.point, buffer, bufferOffset, null);
    // Serialize message field [point_number]
    bufferOffset = _serializer.float64(obj.point_number, buffer, bufferOffset);
    // Serialize message field [end_track]
    bufferOffset = _serializer.bool(obj.end_track, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryPoint
    let len;
    let data = new TrajectoryPoint(null);
    // Deserialize message field [point]
    data.point = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [point_number]
    data.point_number = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [end_track]
    data.end_track = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.point.length;
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'msgs_navigation/TrajectoryPoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6e04038e6f3b9fe907134b6ecbe58bb7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Clothoid data
    
    float64[] point
    
    uint8 X = 0
    uint8 Y = 1
    uint8 KAPPA = 2
    uint8 ANGLE = 3
    uint8 LENGTH = 4
    uint8 SPEED = 5
    uint8 KAPPA_DERIVATIVE = 6
    
    float64 point_number
    bool end_track
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajectoryPoint(null);
    if (msg.point !== undefined) {
      resolved.point = msg.point;
    }
    else {
      resolved.point = []
    }

    if (msg.point_number !== undefined) {
      resolved.point_number = msg.point_number;
    }
    else {
      resolved.point_number = 0.0
    }

    if (msg.end_track !== undefined) {
      resolved.end_track = msg.end_track;
    }
    else {
      resolved.end_track = false
    }

    return resolved;
    }
};

// Constants for message
TrajectoryPoint.Constants = {
  X: 0,
  Y: 1,
  KAPPA: 2,
  ANGLE: 3,
  LENGTH: 4,
  SPEED: 5,
  KAPPA_DERIVATIVE: 6,
}

module.exports = TrajectoryPoint;
