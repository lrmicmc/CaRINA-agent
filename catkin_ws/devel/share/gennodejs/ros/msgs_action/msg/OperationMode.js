// Auto-generated. Do not edit!

// (in-package msgs_action.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class OperationMode {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OperationMode
    // Serialize message field [value]
    bufferOffset = _serializer.int32(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OperationMode
    let len;
    let data = new OperationMode(null);
    // Deserialize message field [value]
    data.value = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'msgs_action/OperationMode';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '77d89d0ca02e0b8d1ca091934c6fbbea';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ART Navigator behaviors (lower numbers have higher priority)
    # $Id: Behavior.msg 996 2011-02-27 16:07:34Z jack.oquin $
    
    #Emergency state
    int32 operational = 0
    int32 emergency = 1
    
    #Acceleration state
    int32 manual_acceleration = 0
    int32 auto_acceleration = 2
    
    #Steering state
    int32 manual_steering = 0
    int32 auto_steering = 4
    
    int32 value
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OperationMode(null);
    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0
    }

    return resolved;
    }
};

// Constants for message
OperationMode.Constants = {
  OPERATIONAL: 0,
  EMERGENCY: 1,
  MANUAL_ACCELERATION: 0,
  AUTO_ACCELERATION: 2,
  MANUAL_STEERING: 0,
  AUTO_STEERING: 4,
}

module.exports = OperationMode;
