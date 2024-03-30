#include "ModbusGateway.h"

ModbusGateway::ModbusGateway(HardwareSerial& serial, uint8_t dePin, uint16_t port) : _server(port, MODBUS_TCP_MAX_CLIENTS) {
  _serial = &serial;
  _dePin = dePin;
}

void ModbusGateway::setTimeout(unsigned long timeout) {
  _responseTimeout = timeout;
}

void ModbusGateway::begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin, bool invert) {
  _server.begin();
  _calculateTimeouts(baud, config);
  _serial->begin(baud, config, rxPin, txPin, invert);
  if (_dePin != NO_DE_PIN) {
    pinMode(_dePin, OUTPUT);
    digitalWrite(_dePin, LOW);
  }
  _clearRTURxBuffer();
}

void ModbusGateway::poll() {
  for (uint8_t i = 0; i < MODBUS_TCP_MAX_CLIENTS; i++) {
    if (!_client[i]) {
      _client[i] = _server.available();
      continue;
    }
    if (!_client[i].connected() || !_client[i].available()) continue;
    uint16_t len = _readTCPRequest(i);
    if (len < 12 || _rtuBuf[0] > 247) continue;
    _writeRTURequest(len - 6);
    if (_rtuBuf[0] == 0) continue;
    len = _readRTUResponse();
    if (len == 0) _writeTCPExceptionResponse(i, 0x0B);
    else _writeTCPResponse(i, len + 6);
  }
}

bool ModbusGateway::getTimeoutFlag() {
  return _timeoutFlag;
}

void ModbusGateway::clearTimeoutFlag() {
  _timeoutFlag = 0;
}



uint16_t ModbusGateway::_readTCPRequest(uint8_t clientId) {
  uint16_t numBytes = 0;
  while (_client[clientId].available() && numBytes < MODBUS_TCP_BUF_SIZE) {
    _tcpBuf[numBytes] = _client[clientId].read();
    numBytes++;
  }
  return numBytes;
}

void ModbusGateway::_writeTCPResponse(uint8_t clientId, uint16_t len) {
  _tcpBuf[4] = highByte(len - 6);
  _tcpBuf[5] = lowByte(len - 6);
  _client[clientId].write(_tcpBuf, len);
  _client[clientId].flush();
}

void ModbusGateway::_writeTCPExceptionResponse(uint8_t clientId, uint8_t code) {
  _tcpBuf[7] |= 0x80;
  _tcpBuf[8] = code;
  _writeTCPResponse(clientId, 9);
}



void ModbusGateway::_writeRTURequest(uint8_t len) {
  uint16_t crc = _crc(len);
  _rtuBuf[len] = lowByte(crc);
  _rtuBuf[len + 1] = highByte(crc);
  digitalWrite(_dePin, HIGH);
  _serial->write(_rtuBuf, len + 2);
  _serial->flush();
  digitalWrite(_dePin, LOW);
}

uint16_t ModbusGateway::_readRTUResponse() {
  unsigned long startTime = millis();
  uint16_t numBytes = 0;
  while (!_serial->available()) {
    if (millis() - startTime >= _responseTimeout) {
      _timeoutFlag = true;
      return 0;
    }
  }
  do {
    if (_serial->available()) {
      startTime = micros();
      _rtuBuf[numBytes] = _serial->read();
      numBytes++;
    }
  } while (micros() - startTime <= _charTimeout && numBytes < MODBUS_RTU_BUF_SIZE);
  while (micros() - startTime < _frameTimeout);
  if (_serial->available() || _crc(numBytes - 2) != _bytesToWord(_rtuBuf[numBytes - 1], _rtuBuf[numBytes - 2])) return 0;
  return (numBytes - 2);
}

void ModbusGateway::_clearRTURxBuffer() {
  unsigned long startTime = micros();
  do {
    if (_serial->available() > 0) {
      startTime = micros();
      _serial->read();
    }
  } while (micros() - startTime < _frameTimeout);
}



void ModbusGateway::_calculateTimeouts(unsigned long baud, uint32_t config) {
  unsigned long bitsPerChar;
  if (config == SERIAL_8E2 || config == SERIAL_8O2) bitsPerChar = 12;
  else if (config == SERIAL_8N2 || config == SERIAL_8E1 || config == SERIAL_8O1) bitsPerChar = 11;
  else bitsPerChar = 10;
  if (baud <= 19200) {
    _charTimeout = (bitsPerChar * 2500000) / baud;
    _frameTimeout = (bitsPerChar * 4500000) / baud;
  }
  else {
    _charTimeout = (bitsPerChar * 1000000) / baud + 750;
    _frameTimeout = (bitsPerChar * 1000000) / baud + 1750;
  }
}

uint16_t ModbusGateway::_crc(uint8_t len) {
  uint16_t value = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    value ^= (uint16_t)_rtuBuf[i];
    for (uint8_t j = 0; j < 8; j++) {
      bool lsb = value & 1;
      value >>= 1;
      if (lsb == true) value ^= 0xA001;
    }
  }
  return value;
}

uint16_t ModbusGateway::_bytesToWord(uint8_t high, uint8_t low) {
  return (high << 8) | low;
}