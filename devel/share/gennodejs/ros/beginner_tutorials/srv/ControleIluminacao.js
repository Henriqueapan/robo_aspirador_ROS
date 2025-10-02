// Auto-generated. Do not edit!

// (in-package beginner_tutorials.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ControleIluminacaoRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.comando = null;
    }
    else {
      if (initObj.hasOwnProperty('comando')) {
        this.comando = initObj.comando
      }
      else {
        this.comando = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControleIluminacaoRequest
    // Serialize message field [comando]
    bufferOffset = _serializer.string(obj.comando, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControleIluminacaoRequest
    let len;
    let data = new ControleIluminacaoRequest(null);
    // Deserialize message field [comando]
    data.comando = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.comando);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'beginner_tutorials/ControleIluminacaoRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '20735899cef58c19fdb13abb7f9d907e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string comando
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControleIluminacaoRequest(null);
    if (msg.comando !== undefined) {
      resolved.comando = msg.comando;
    }
    else {
      resolved.comando = ''
    }

    return resolved;
    }
};

class ControleIluminacaoResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.resposta = null;
    }
    else {
      if (initObj.hasOwnProperty('resposta')) {
        this.resposta = initObj.resposta
      }
      else {
        this.resposta = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControleIluminacaoResponse
    // Serialize message field [resposta]
    bufferOffset = _serializer.string(obj.resposta, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControleIluminacaoResponse
    let len;
    let data = new ControleIluminacaoResponse(null);
    // Deserialize message field [resposta]
    data.resposta = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.resposta);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'beginner_tutorials/ControleIluminacaoResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '59415f2556b55ab9879d7fd7b2bbab3b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string resposta
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControleIluminacaoResponse(null);
    if (msg.resposta !== undefined) {
      resolved.resposta = msg.resposta;
    }
    else {
      resolved.resposta = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: ControleIluminacaoRequest,
  Response: ControleIluminacaoResponse,
  md5sum() { return 'dd563be63c4578244cd65e069bc1911e'; },
  datatype() { return 'beginner_tutorials/ControleIluminacao'; }
};
