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
 * License along with GPRSbee.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <Stream.h>

enum {
  XRF_OK = 0,
  XRF_TIMEOUT,
  XRF_MAXRETRY,
  XRF_UNKNOWN,
};

class XRF
{
public:
  void init(uint16_t devID, Stream &stream,
      uint8_t nrRetries=3, uint16_t retryTimeout=1000);
  uint8_t setPanID(uint16_t panID);
  uint16_t getPanID();
  uint8_t setBaudRate(uint32_t rate);
  uint8_t setATBD(uint16_t rate);
  uint32_t getBaudRate();

  void setDiag(Stream &stream) { _diagStream = &stream; }

private:
  bool readLine(char *buffer, size_t size, uint16_t timeout=2000);
  void flushInput();
  bool waitForOK(uint16_t timeout=2000);
  void sendCommand(const char *cmd);
  uint8_t sendCommandWaitForOK(const char *cmd, uint16_t timeout=2000);

  uint8_t enterCmndMode();
  uint8_t leaveCmndMode();

  // Small utility to see if we timed out
  static bool isTimedOut(uint32_t ts) { return (long)(millis() - ts) >= 0; }

  void diagPrint(const char *str) { if (_diagStream) _diagStream->print(str); }
  void diagPrintLn(const char *str) { if (_diagStream) _diagStream->println(str); }
  void diagPrint(const __FlashStringHelper *str) { if (_diagStream) _diagStream->print(str); }
  void diagPrintLn(const __FlashStringHelper *str) { if (_diagStream) _diagStream->println(str); }
  void diagPrint(int i) { if (_diagStream) _diagStream->print(i); }
  void diagPrintLn(int i) { if (_diagStream) _diagStream->println(i); }
  void diagPrint(char c) { if (_diagStream) _diagStream->print(c); }
  void diagPrintLn(char c) { if (_diagStream) _diagStream->println(c); }

private:
  Stream *_myStream;
  Stream *_diagStream;
  uint16_t _devID;
  uint16_t _panID;
  uint8_t _nrRetries;
  uint16_t _retryTimeout;
};

#endif  /* XRF_H_ */
