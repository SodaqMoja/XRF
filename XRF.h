#ifndef XFR_H_
#define XRF_H_
/*
 * Copyright (c) 2013-2014 Kees Bakker.  All rights reserved.
 *
 * This file is part of the XRF library for Arduino.
 *
 * GPRSbee is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or(at your option) any later version.
 *
 * GPRSbee is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with the XRF library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <Arduino.h>

/*
 * Make this an undef (or comment) to disable diagnostic
 */
#define XRF_ENABLE_DIAG 1

enum {
  XRF_OK = 0,
  XRF_NOT_OK,                   // Failed to see OK
  XRF_NACK,
  XRF_TIMEOUT,
  XRF_MAXRETRY,
  XRF_CRC_ERROR,
  XRF_FAIL_CMDMODE,             // Failed to enter command mode (+++)
  XRF_NOT_IN_CMDMODE,           // Attempting a command while not in Command Mode
  XRF_NOT_IMPLEMENTED,          // Function not yet implemented
  XRF_UNKNOWN,
};

class XRF
{
public:
  XRF();
  XRF(Stream &stream,
      uint8_t nrRetries=3, uint16_t retryTimeout=1000);
  XRF(Stream &stream,
      const char *devName,
      uint16_t panID,
      uint8_t nrRetries=3, uint16_t retryTimeout=1000);

  void config(uint8_t dataRate, uint8_t packetSize=0, uint16_t packetTimeout=0);

  void init();
  void init(const char *devName);
  void setFldSep(char fldSep) { _fldSep = fldSep; }

  uint8_t leaveCmndMode();

  uint8_t setPanID(uint16_t panID);
  uint16_t getPanID();

  uint8_t setBaudRate(uint32_t rate);
  uint8_t setATBD(uint32_t rate) { return setBaudRate(rate); }
  uint32_t getBaudRate();

  uint8_t setDataRate(uint8_t rate);
  uint8_t setATDR(uint8_t rate) { return setDataRate(rate); }
  uint8_t getDataRate();

  uint8_t setPacketSize(uint8_t size);
  uint8_t setATPK(uint8_t size) { return setPacketSize(size); }
  uint8_t getPacketSize();

  uint8_t setPacketTimeout(uint16_t timeout);
  uint8_t setATRO(uint16_t timeout) { return setPacketTimeout(timeout); }
  uint16_t getPacketTimeout();

  uint8_t setSleepMode(uint8_t mode, uint8_t sleepPin);
  void sleep();
  void wakeUp();

  void flushInput();

  uint8_t sendData(const char *dest, const char *data);
  uint8_t receiveData(char *source, size_t sourceSize, char *data, size_t dataSize, uint16_t timeout=3000);
  int available() { return _myStream->available(); }

  uint8_t sendDataAndWaitForReply(const char *dest, const char *data, char *reply, size_t replySize);

  size_t getFailedCounter() { return _failedCounter; }

  void setDiag(Stream &stream) { _diagStream = &stream; }

private:
  void sendDataNoWait(const char *dest, const char *data);
  uint8_t receiveDataNoAck(char *source, size_t sourceSize, char *data, size_t dataSize, uint16_t timeout=3000);
  uint8_t waitForReply(char *reply, size_t replySize);
  uint8_t waitForAck();

  bool readLine(char *buffer, size_t size, uint16_t timeout=2000);
  uint8_t waitForOK(uint16_t timeout=2000);
  void sendCommand(const char *cmd);
  uint8_t sendCommandWaitForOK(const char *cmd, uint16_t timeout=2000);
  bool sendATxGetHexNumber(const char *at, uint32_t *num);
  uint8_t sendATxSetHexNumber(const char *at, uint32_t num);

  uint8_t enterCmndMode();

  // Small utility to see if we timed out
  static bool isTimedOut(uint32_t ts) { return (long)(millis() - ts) >= 0; }

  bool findCrc(char *txt, uint16_t *crc, char **cptr);
  uint16_t crc16_ccitt(uint8_t * buf, size_t len);
  uint16_t crc16_xmodem(uint8_t * buf, size_t len);

  // TODO There must be an easier way to do this.
#if XRF_ENABLE_DIAG
  void diagPrint(const char *str) { if (_diagStream) _diagStream->print(str); }
  void diagPrintLn(const char *str) { if (_diagStream) _diagStream->println(str); }
  void diagPrint(const __FlashStringHelper *str) { if (_diagStream) _diagStream->print(str); }
  void diagPrintLn(const __FlashStringHelper *str) { if (_diagStream) _diagStream->println(str); }
  void diagPrint(int i, int base=DEC) { if (_diagStream) _diagStream->print(i, base); }
  void diagPrintLn(int i, int base=DEC) { if (_diagStream) _diagStream->println(i, base); }
  void diagPrint(unsigned int u, int base=DEC) { if (_diagStream) _diagStream->print(u, base); }
  void diagPrintLn(unsigned int u, int base=DEC) { if (_diagStream) _diagStream->println(u, base); }
  void diagPrint(char c) { if (_diagStream) _diagStream->print(c); }
  void diagPrintLn(char c) { if (_diagStream) _diagStream->println(c); }
#else
  void diagPrint(const char *str) {}
  void diagPrintLn(const char *str) {}
  void diagPrint(const __FlashStringHelper *str) {}
  void diagPrintLn(const __FlashStringHelper *str) {}
  void diagPrint(int i, int base=DEC) {}
  void diagPrintLn(int i, int base=DEC) {}
  void diagPrint(unsigned int u, int base=DEC) {}
  void diagPrintLn(unsigned int u, int base=DEC) {}
  void diagPrint(char c) {}
  void diagPrintLn(char c) {}
#endif

private:
  Stream *_myStream;
  char _eol;
  Stream *_diagStream;
  uint16_t _panID;
  uint8_t _nrRetries;
  uint16_t _retryTimeout;
  char _fldSep;
  bool _inCmndMode;
  uint8_t _sleepPin;
  uint8_t _sleepMode;
  char _devName[10];
  size_t _failedCounter;
};

#endif  /* XRF_H_ */
