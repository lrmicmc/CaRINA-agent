// Auto-generated. Do not edit!

// (in-package msgs_perception.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class BoundingBox {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.classe = null;
      this.p1 = null;
      this.p2 = null;
      this.p3 = null;
      this.p4 = null;
      this.probability = null;
    }
    else {
      if (initObj.hasOwnProperty('classe')) {
        this.classe = initObj.classe
      }
      else {
        this.classe = new std_msgs.msg.String();
      }
      if (initObj.hasOwnProperty('p1')) {
        this.p1 = initObj.p1
      }
      else {
        this.p1 = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('p2')) {
        this.p2 = initObj.p2
      }
      else {
        this.p2 = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('p3')) {
        this.p3 = initObj.p3
      }
      else {
        this.p3 = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('p4')) {
        this.p4 = initObj.p4
      }
      else {
        this.p4 = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('probability')) {
        this.probability = initObj.probability
      }
      else {
        this.probability = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BoundingBox
    // Serialize message field [classe]
    bufferOffset = std_msgs.msg.String.serialize(obj.classe, buffer, bufferOffset);
    // Serialize message field [p1]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.p1, buffer, bufferOffset);
    // Serialize message field [p2]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.p2, buffer, bufferOffset);
    // Serialize message field [p3]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.p3, buffer, bufferOffset);
    // Serialize message field [p4]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.p4, buffer, bufferOffset);
    // Serialize message field [probability]
    bufferOffset = _serializer.float64(obj.probability, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BoundingBox
    let len;
    let data = new BoundingBox(null);
    // Deserialize message field [classe]
    data.classe = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [p1]
    data.p1 = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [p2]
    data.p2 = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [p3]
    data.p3 = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [p4]
    data.p4 = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [probability]
    data.probability = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.classe);
    return length + 104;
  }

  static datatype() {
    // Returns string type for a message object
    return 'msgs_perception/BoundingBox';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7f559df2715f948d4d334146387c4e69';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BoundingBox(null);
    if (msg.classe !== undefined) {
      resolved.classe = std_msgs.msg.String.Resolve(msg.classe)
    }
    else {
      resolved.classe = new std_msgs.msg.String()
    }

    if (msg.p1 !== undefined) {
      resolved.p1 = geometry_msgs.msg.Point.Resolve(msg.p1)
    }
    else {
      resolved.p1 = new geometry_msgs.msg.Point()
    }

    if (msg.p2 !== undefined) {
      resolved.p2 = geometry_msgs.msg.Point.Resolve(msg.p2)
    }
    else {
      resolved.p2 = new geometry_msgs.msg.Point()
    }

    if (msg.p3 !== undefined) {
      resolved.p3 = geometry_msgs.msg.Point.Resolve(msg.p3)
    }
    else {
      resolved.p3 = new geometry_msgs.msg.Point()
    }

    if (msg.p4 !== undefined) {
      resolved.p4 = geometry_msgs.msg.Point.Resolve(msg.p4)
    }
    else {
      resolved.p4 = new geometry_msgs.msg.Point()
    }

    if (msg.probability !== undefined) {
      resolved.probability = msg.probability;
    }
    else {
      resolved.probability = 0.0
    }

    return resolved;
    }
};

module.exports = BoundingBox;
