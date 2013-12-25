#ifndef XFR_H_
#define XRF_H_
/*
 * Copyright (c) 2013 Kees Bakker.  All rights reserved.
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
#include <Stream.h>

enum {
  XRF_OK = 0,
  XRF_NOT_OK,                   // Failed to see OK
  XRF_TIMEOUT,
  XRF_MAXRETRY,
  XRF_FAIL_CMDMODE,             // Failed to enter command mode (+++)
  XRF_NOT_IN_CMDMODE,           // Attempting a command while not in Command Mode
  XRF_NOT_IMPLEMENTED,          // Function not yet implemented
  XRF_UNKNOWN,
};

class XRF
{
public:
  XRF();

  void init(Stream &stream,
      uint8_t nrRetries=3, uint16_t retryTimeout=1000);

  uint8_t leaveCmndMode();

  uint8_t setPanID(uint16_t panID);
  uint16_t getPanID();

  uint8_t setBaudRate(uint32_t rate);
  uint8_t setATBD(uint32_t rate) { return setBaudRate(rate); }
  uint32_t getBaudRate();

  uint8_t setDataRate(uint8_t rate);
  uint8_t setATDR(uint8_t rate) { return setDataRate(rate); }
  uint8_t getDataRate();

  uint8_t sendData(const char *data, uint16_t timeout=3000);
  uint8_t receiveData(const char *prefix, char *data, size_t size, uint16_t timeout=3000);
  uint8_t receiveData(char *data, size_t size, uint16_t timeout=3000);
  int available() { return _myStream->available(); }

  void setDiag(Stream &stream) { _diagStream = &stream; }

private:
  bool readLine(char *buffer, size_t size, uint16_t timeout=2000);
  void flushInput();
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

  // TODO There must be an easier way to do this.
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

private:
  Stream *_myStream;
  char _eol;
  Stream *_diagStream;
  uint16_t _panID;
  uint8_t _nrRetries;
  uint16_t _retryTimeout;
  bool _inCmndMode;
};

#endif  /* XRF_H_ */
