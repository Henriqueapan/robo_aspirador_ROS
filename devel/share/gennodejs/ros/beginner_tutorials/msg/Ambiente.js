// Auto-generated. Do not edit!

// (in-package beginner_tutorials.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Ambiente {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.temperatura = null;
      this.umidade = null;
    }
    else {
      if (initObj.hasOwnProperty('temperatura')) {
        this.temperatura = initObj.temperatura
      }
      else {
        this.temperatura = 0.0;
      }
      if (initObj.hasOwnProperty('umidade')) {
        this.umidade = initObj.umidade
      }
      else {
        this.umidade = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Ambiente
    // Serialize message field [temperatura]
    bufferOffset = _serializer.float32(obj.temperatura, buffer, bufferOffset);
    // Serialize message field [umidade]
    bufferOffset = _serializer.float32(obj.umidade, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Ambiente
    let len;
    let data = new Ambiente(null);
    // Deserialize message field [temperatura]
    data.temperatura = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [umidade]
    data.umidade = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'beginner_tutorials/Ambiente';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '34ba80ca03e34a8cd5a9afb761234652';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 temperatura
    float32 umidade
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Ambiente(null);
    if (msg.temperatura !== undefined) {
      resolved.temperatura = msg.temperatura;
    }
    else {
      resolved.temperatura = 0.0
    }

    if (msg.umidade !== undefined) {
      resolved.umidade = msg.umidade;
    }
    else {
      resolved.umidade = 0.0
    }

    return resolved;
    }
};

module.exports = Ambiente;
