// Auto-generated. Do not edit!

// (in-package msgs_action.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ackermann_msgs = _finder('ackermann_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class VehicleState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drive = null;
      this.engine_speed = null;
      this.throttle = null;
      this.car_gear = null;
      this.brake = null;
      this.handbrake = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('drive')) {
        this.drive = initObj.drive
      }
      else {
        this.drive = new ackermann_msgs.msg.AckermannDrive();
      }
      if (initObj.hasOwnProperty('engine_speed')) {
        this.engine_speed = initObj.engine_speed
      }
      else {
        this.engine_speed = 0;
      }
      if (initObj.hasOwnProperty('throttle')) {
        this.throttle = initObj.throttle
      }
      else {
        this.throttle = 0.0;
      }
      if (initObj.hasOwnProperty('car_gear')) {
        this.car_gear = initObj.car_gear
      }
      else {
        this.car_gear = 0;
      }
      if (initObj.hasOwnProperty('brake')) {
        this.brake = initObj.brake
      }
      else {
        this.brake = 0;
      }
      if (initObj.hasOwnProperty('handbrake')) {
        this.handbrake = initObj.handbrake
      }
      else {
        this.handbrake = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VehicleState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drive]
    bufferOffset = ackermann_msgs.msg.AckermannDrive.serialize(obj.drive, buffer, bufferOffset);
    // Serialize message field [engine_speed]
    bufferOffset = _serializer.int32(obj.engine_speed, buffer, bufferOffset);
    // Serialize message field [throttle]
    bufferOffset = _serializer.float64(obj.throttle, buffer, bufferOffset);
    // Serialize message field [car_gear]
    bufferOffset = _serializer.int32(obj.car_gear, buffer, bufferOffset);
    // Serialize message field [brake]
    bufferOffset = _serializer.int32(obj.brake, buffer, bufferOffset);
    // Serialize message field [handbrake]
    bufferOffset = _serializer.int32(obj.handbrake, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VehicleState
    let len;
    let data = new VehicleState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drive]
    data.drive = ackermann_msgs.msg.AckermannDrive.deserialize(buffer, bufferOffset);
    // Deserialize message field [engine_speed]
    data.engine_speed = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [throttle]
    data.throttle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [car_gear]
    data.car_gear = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [brake]
    data.brake = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [handbrake]
    data.handbrake = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'msgs_action/VehicleState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd59fbb8f50f426518dc752cfb6444965';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    ackermann_msgs/AckermannDrive drive	# Look for ackermann_msgs at ros wiki
    int32 engine_speed					# Engine revolution-counter [range 0 to 10240 rpm]
    float64 throttle					# Throttle pedal position [range: 0 to 99.96%]
    int32 car_gear 						# Gear selection [range: -1 (reverse), 0 (neutral) to 5 (fifth)]
    int32 brake							# Brake active (1) or inactive (0)
    int32 handbrake						# Handbrake active (1) or inactive (0) 
    
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
    MSG: ackermann_msgs/AckermannDrive
    ## Driving command for a car-like vehicle using Ackermann steering.
    #  $Id$
    
    # Assumes Ackermann front-wheel steering. The left and right front
    # wheels are generally at different angles. To simplify, the commanded
    # angle corresponds to the yaw of a virtual wheel located at the
    # center of the front axle, like on a tricycle.  Positive yaw is to
    # the left. (This is *not* the angle of the steering wheel inside the
    # passenger compartment.)
    #
    # Zero steering angle velocity means change the steering angle as
    # quickly as possible. Positive velocity indicates a desired absolute
    # rate of change either left or right. The controller tries not to
    # exceed this limit in either direction, but sometimes it might.
    #
    float32 steering_angle          # desired virtual angle (radians)
    float32 steering_angle_velocity # desired rate of change (radians/s)
    
    # Drive at requested speed, acceleration and jerk (the 1st, 2nd and
    # 3rd derivatives of position). All are measured at the vehicle's
    # center of rotation, typically the center of the rear axle. The
    # controller tries not to exceed these limits in either direction, but
    # sometimes it might.
    #
    # Speed is the desired scalar magnitude of the velocity vector.
    # Direction is forward unless the sign is negative, indicating reverse.
    #
    # Zero acceleration means change speed as quickly as
    # possible. Positive acceleration indicates a desired absolute
    # magnitude; that includes deceleration.
    #
    # Zero jerk means change acceleration as quickly as possible. Positive
    # jerk indicates a desired absolute rate of acceleration change in
    # either direction (increasing or decreasing).
    #
    float32 speed                   # desired forward speed (m/s)
    float32 acceleration            # desired acceleration (m/s^2)
    float32 jerk                    # desired jerk (m/s^3)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VehicleState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.drive !== undefined) {
      resolved.drive = ackermann_msgs.msg.AckermannDrive.Resolve(msg.drive)
    }
    else {
      resolved.drive = new ackermann_msgs.msg.AckermannDrive()
    }

    if (msg.engine_speed !== undefined) {
      resolved.engine_speed = msg.engine_speed;
    }
    else {
      resolved.engine_speed = 0
    }

    if (msg.throttle !== undefined) {
      resolved.throttle = msg.throttle;
    }
    else {
      resolved.throttle = 0.0
    }

    if (msg.car_gear !== undefined) {
      resolved.car_gear = msg.car_gear;
    }
    else {
      resolved.car_gear = 0
    }

    if (msg.brake !== undefined) {
      resolved.brake = msg.brake;
    }
    else {
      resolved.brake = 0
    }

    if (msg.handbrake !== undefined) {
      resolved.handbrake = msg.handbrake;
    }
    else {
      resolved.handbrake = 0
    }

    return resolved;
    }
};

module.exports = VehicleState;
