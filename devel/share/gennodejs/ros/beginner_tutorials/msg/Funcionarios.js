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

class Funcionarios {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.nome = null;
      this.idade = null;
      this.cargo = null;
      this.altura = null;
    }
    else {
      if (initObj.hasOwnProperty('nome')) {
        this.nome = initObj.nome
      }
      else {
        this.nome = '';
      }
      if (initObj.hasOwnProperty('idade')) {
        this.idade = initObj.idade
      }
      else {
        this.idade = 0;
      }
      if (initObj.hasOwnProperty('cargo')) {
        this.cargo = initObj.cargo
      }
      else {
        this.cargo = '';
      }
      if (initObj.hasOwnProperty('altura')) {
        this.altura = initObj.altura
      }
      else {
        this.altura = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Funcionarios
    // Serialize message field [nome]
    bufferOffset = _serializer.string(obj.nome, buffer, bufferOffset);
    // Serialize message field [idade]
    bufferOffset = _serializer.int8(obj.idade, buffer, bufferOffset);
    // Serialize message field [cargo]
    bufferOffset = _serializer.string(obj.cargo, buffer, bufferOffset);
    // Serialize message field [altura]
    bufferOffset = _serializer.float64(obj.altura, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Funcionarios
    let len;
    let data = new Funcionarios(null);
    // Deserialize message field [nome]
    data.nome = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [idade]
    data.idade = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [cargo]
    data.cargo = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [altura]
    data.altura = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.nome);
    length += _getByteLength(object.cargo);
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'beginner_tutorials/Funcionarios';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f599a6c815b91d516a23de2962c57921';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string nome
    int8 idade
    string cargo
    float64 altura
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Funcionarios(null);
    if (msg.nome !== undefined) {
      resolved.nome = msg.nome;
    }
    else {
      resolved.nome = ''
    }

    if (msg.idade !== undefined) {
      resolved.idade = msg.idade;
    }
    else {
      resolved.idade = 0
    }

    if (msg.cargo !== undefined) {
      resolved.cargo = msg.cargo;
    }
    else {
      resolved.cargo = ''
    }

    if (msg.altura !== undefined) {
      resolved.altura = msg.altura;
    }
    else {
      resolved.altura = 0.0
    }

    return resolved;
    }
};

module.exports = Funcionarios;
