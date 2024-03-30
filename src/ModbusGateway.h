#ifndef ModbusGateway_h
#define ModbusGateway_h

#define MODBUS_RTU_BUF_SIZE 256
#define MODBUS_TCP_BUF_SIZE (MODBUS_RTU_BUF_SIZE + 6)
#define NO_DE_PIN 255
#define MODBUS_TCP_PORT 502
#define MODBUS_TCP_MAX_CLIENTS 4

#include "Arduino.h"
#include <WiFi.h>

class ModbusGateway {
  public:
    ModbusGateway(HardwareSerial& serial, uint8_t dePin = NO_DE_PIN, uint16_t port = MODBUS_TCP_PORT);
    void setTimeout(unsigned long timeout);
    void begin(unsigned long baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1, bool invert = false);
    void poll();
    bool getTimeoutFlag();
    void clearTimeoutFlag();

  private:
    WiFiServer _server;
    WiFiClient _client[MODBUS_TCP_MAX_CLIENTS];
    HardwareSerial *_serial;
    uint8_t _dePin;
    uint8_t _tcpBuf[MODBUS_TCP_BUF_SIZE];
    uint8_t *_rtuBuf = _tcpBuf + 6;
    unsigned long _charTimeout;
    unsigned long _frameTimeout;
    unsigned long _responseTimeout = 100;
    bool _timeoutFlag = false;
    
    uint16_t _readTCPRequest(uint8_t clientId);
    void _writeTCPResponse(uint8_t clientId, uint16_t len);
    void _writeTCPExceptionResponse(uint8_t clientId, uint8_t code);

    void _writeRTURequest(uint8_t len);
    uint16_t _readRTUResponse();
    void _clearRTURxBuffer();

    void _calculateTimeouts(unsigned long baud, uint32_t config);
    uint16_t _crc(uint8_t len);
    uint16_t _bytesToWord(uint8_t high, uint8_t low);
};

#endif