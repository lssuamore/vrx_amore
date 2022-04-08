// Auto-generated. Do not edit!

// (in-package amore.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let NED_buoy = require('./NED_buoy.js');

//-----------------------------------------------------------

class NED_buoys {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.buoys = null;
      this.quantity = null;
    }
    else {
      if (initObj.hasOwnProperty('buoys')) {
        this.buoys = initObj.buoys
      }
      else {
        this.buoys = [];
      }
      if (initObj.hasOwnProperty('quantity')) {
        this.quantity = initObj.quantity
      }
      else {
        this.quantity = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NED_buoys
    // Serialize message field [buoys]
    // Serialize the length for message field [buoys]
    bufferOffset = _serializer.uint32(obj.buoys.length, buffer, bufferOffset);
    obj.buoys.forEach((val) => {
      bufferOffset = NED_buoy.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [quantity]
    bufferOffset = _serializer.int32(obj.quantity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NED_buoys
    let len;
    let data = new NED_buoys(null);
    // Deserialize message field [buoys]
    // Deserialize array length for message field [buoys]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.buoys = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.buoys[i] = NED_buoy.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [quantity]
    data.quantity = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.buoys.forEach((val) => {
      length += NED_buoy.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'amore/NED_buoys';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '770b53fd7910d5fb852692f8fc415cee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    amore/NED_buoy[] buoys
    int32 quantity
    
    ================================================================================
    MSG: amore/NED_buoy
    geometry_msgs/Point position
    string id
    
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
    const resolved = new NED_buoys(null);
    if (msg.buoys !== undefined) {
      resolved.buoys = new Array(msg.buoys.length);
      for (let i = 0; i < resolved.buoys.length; ++i) {
        resolved.buoys[i] = NED_buoy.Resolve(msg.buoys[i]);
      }
    }
    else {
      resolved.buoys = []
    }

    if (msg.quantity !== undefined) {
      resolved.quantity = msg.quantity;
    }
    else {
      resolved.quantity = 0
    }

    return resolved;
    }
};

module.exports = NED_buoys;
