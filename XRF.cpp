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
#include <string.h>
#include <util/crc16.h>
#include <Arduino.h>
#include "XRF.h"

XRF::XRF()
{
  _myStream = 0;
  _eol = '\n';
  _diagStream = 0;
  _panID = 0;
  _nrRetries = 3;
  _retryTimeout = 1000;
  _fldSep = ',';
  _inCmndMode = false;
  _sleepPin = 0;
  _sleepPinStatus = 0xFF;
  _sleepMode = 0xFF;
  _sleepDelay = 1000;
  memset(_devName, 0, sizeof(_devName));
  _failedCounter = 0;
  _setNow = 0;
  _getNow = 0;
}

XRF::XRF(Stream &stream,
    uint16_t panID,
    uint8_t nrRetries, uint16_t retryTimeout)
{
  _myStream = &stream;
  _eol = '\n';
  _diagStream = 0;
  _panID = panID;
  _nrRetries = nrRetries;
  _retryTimeout = retryTimeout;
  _fldSep = ',';
  _inCmndMode = false;
  _sleepPin = 0;
  _sleepPinStatus = 0xFF;
  _sleepMode = 0xFF;
  _sleepDelay = 1000;
  memset(_devName, 0, sizeof(_devName));
  _failedCounter = 0;
  _setNow = 0;
  _getNow = 0;
}

XRF::XRF(Stream &stream,
    uint16_t panID,
    const char *devName,
    uint8_t nrRetries, uint16_t retryTimeout)
{
  _myStream = &stream;
  _eol = '\n';
  _diagStream = 0;
  _panID = panID;
  _nrRetries = nrRetries;
  _retryTimeout = retryTimeout;
  _fldSep = ',';
  _inCmndMode = false;
  _sleepPin = 0;
  _sleepPinStatus = 0xFF;
  _sleepMode = 0xFF;
  _sleepDelay = 1000;
  memset(_devName, 0, sizeof(_devName));
  strncpy(_devName, devName, sizeof(_devName) - 1);
  _failedCounter = 0;
  _setNow = 0;
  _getNow = 0;
}

/*
 * Configure a few extra parameters
 */
void XRF::config(uint8_t dataRate, uint8_t packetSize, uint16_t packetTimeout)
{
  if (dataRate) {
    setDataRate(dataRate);
    doApplyChanges();
  }
  if (packetSize) {
    setPacketSize(packetSize);
  }
  if (packetTimeout) {
    setPacketTimeout(packetTimeout);
  }
}

/*
* Initialize XRF
*
* Do the initialisation of the device.
*  - set the panID
*  - set the sleepMode
*/
void XRF::init()
{
  // We really must be sure it is not sleeping.
  while (_sleepMode == 0xFF) {
    _sleepMode = getATSM();
    diagPrint(F("XRF::init _sleepMode ")); diagPrintLn(_sleepMode);
  }
  // Setting panID must succeed
  while (_panID && setPanID(_panID) != XRF_OK) {
  }
  (void)leaveCmndMode();
}
void XRF::init(const char *devName)
{
  strncpy(_devName, devName, sizeof(_devName) - 1);
  init();
}

/*
* Send ASCII data via the XRF device
*
* The data is sent as ASCII. The CRC is appended and an end-of-line
* is added too.
*
* \param dest the destination
* \param data the ASCII data
*/
uint8_t XRF::sendData(const char *dest, const char *data)
{
  //diagPrint(F("SendData: dest='")); diagPrint(dest); diagPrint('\'');
  //diagPrint(F(" data='")); diagPrint(data); diagPrintLn('\'');

  // All current input can be flushed. It is not for us.
  flushInput();

  uint8_t status = XRF_UNKNOWN;
  for (size_t i = 0; i < _nrRetries; ++i) {

    sendDataNoWait(dest, data);

    // Wait for an ACK
    status = waitForAck(dest, _retryTimeout);
    if (status == XRF_OK) {
      break;
    }
    if (status == XRF_NACK) {
      break;
    }
    status = XRF_MAXRETRY;
  }
  //diagPrint(F(" status ")); diagPrintLn(status);

  return status;
}

/*
* Send ASCII data via the XRF device
*
* The data is sent as ASCII. The CRC is appended and an end-of-line
* is added too.
*
* \param dest the destination
* \param data the ASCII data
*/
void XRF::sendDataNoWait(const char *dest, const char *data)
{
  //diagPrint(F("SendDataNoWait: dest='")); diagPrint(dest); diagPrint('\'');
  //diagPrint(F(" data='")); diagPrint(data); diagPrintLn('\'');

  String line;
  line.reserve(60);
  line += dest;
  line += _fldSep;
  line += _devName;
  line += _fldSep;
  line += data;
  const char *cstr = line.c_str();
  uint16_t crc = crc16_xmodem((uint8_t *)cstr, line.length());
  line += _fldSep;
  line += crc;

  _myStream->print(cstr);
  _myStream->print(_eol);
}

/*
 * Wait for an "ack"
 *
 * The syntax of the ack message (after having stripped <dest> and <crc>) is:
 *   <source> ',' "ack"
 * We must check that the ack is indeed from the expected source.
 *
 * In our embedded environment we have to worry about stack space. In this
 * function we create an area on stack that is used to receive the whole
 * line with the "ack". It must be big enough to hold everything on the line:
 *   <dest> ',' <source> ',' <ts> ',' "ack" ',' <crc>
 * (Should we have <ts> to be optional?)
 */
uint8_t XRF::waitForAck(const char *from, uint16_t timeout)
{
  uint8_t status;
  char reply[40];               // We need space for the WHOLE line (receiveDataNoAck uses it too)
  char source2[12];
  char *cptr;

  status = receiveDataNoAck(source2, sizeof(source2), reply, sizeof(reply), timeout);
  if (status == XRF_OK) {
    // The "reply" buffer contains <ts> ',' "ack", or maybe
    // just "ack"
    status = XRF_NOT_OK;                // Assume the worst
    if (strcmp(source2, from) != 0) {
    } else {
      // Do we have two fields?
      cptr = strchr(reply, _fldSep);
      if (cptr) {
        // First field is assumed to be a timestamp
        *cptr = '\0';          // terminate that string
        if (_setNow) {
          uint32_t ts;
          if (getUValue(reply, &ts)) {
            _setNow(ts);
          }
        }
        ++cptr;                 // Skip the comma
      } else {
        cptr = reply;
      }
      if (strcmp(cptr, "ack") == 0) {
        status = XRF_OK;
      } else if (strcmp(cptr, "nack") == 0) {
        status = XRF_NACK;
      } else {
        // Unknown ack packet
      }
    }
  }

  return status;
}

uint8_t XRF::receiveData(char *source, size_t sourceSize,
    char *data, size_t dataSize, uint16_t timeout)
{
  char source2[12];
  uint8_t status;
  if (source) {
    memset(source, 0, sourceSize);
  }
  status = receiveDataNoAck(source2, sizeof(source2), data, dataSize, timeout);
  if (status == XRF_OK) {
    // Send an ack
    sendAck(source2);
    if (source) {
      // Return the name of the source of the packet.
      strncpy(source, source2, sourceSize - 1);
    }
  }
  return status;
}

/*
 * Wait for a line of text that starts with the prefix
 *
 * \param source a pointer to store the name of the source
 * \param sourceSize the maximum number of bytes to store in the source (including \0)
 * \param data a pointer to store the result
 * \param dataSize the maximum number of bytes to store in the result buffer (including \0)
 * \param timeout the maximum number of milliseconds for the whole operation
 * \return XRF_OK if the operation was successful or else an error code
 */
uint8_t XRF::receiveDataNoAck(char *source, size_t sourceSize,
    char *data, size_t dataSize, uint16_t timeout)
{
  uint8_t status = XRF_TIMEOUT;
  char prefix[16];              // _devName plus comma plus \0
  char *ptr;

  // Prepare the "prefix". The packet must start with this.
  // We use our own address, our own name.
  ptr = prefix;
  strcpy(ptr, _devName);
  ptr += strlen(ptr);
  *ptr++ = _fldSep;
  *ptr = '\0';

  //diagPrint(F("receiveDataNoAck prefix: '")); diagPrint(prefix); diagPrintLn('\'');
  // FIXME The while loop has the same timeout as the readLine call. That's not right.
  uint32_t ts_max = millis() + timeout;
  while (!isTimedOut(ts_max)) {
    if (readLine(data, dataSize, timeout)) {
      //diagPrint(F("receiveDataNoAck data'")); diagPrint(data); diagPrintLn('\'');
      size_t len = strlen(prefix);
      // Does the prefix match?
      if (strncmp(data, prefix, len) == 0) {
        // Yes, it does.

        // Next, verify the checksum
        uint16_t crc;
        char *cptr;
        if (findCrc(data, &crc, &cptr)) {
          //diagPrint(F("receiveData crc: ")); diagPrintLn(crc);

          // Strip the checksum
          *cptr = '\0';
          uint16_t crc1 = crc16_xmodem((uint8_t *)data, strlen(data));
          //diagPrint(F("receiveDataNoAck: '")); diagPrint(data); diagPrintLn('\'');
          //diagPrint(F("receiveDataNoAck checksum : ")); diagPrintLn(crc == crc1 ? "OK" : "not OK");
          if (crc1 == crc) {
            // Yes, the checksum is correct.
            _failedCounter = 0;

            // Strip the prefix from the reply
            strcpy(data, data + len);
            //diagPrint(F("receiveDataNoAck: 2 '")); diagPrint(data); diagPrintLn('\'');

            // Now, the first field is the sender address
            cptr = strchr(data, _fldSep);
            if (cptr != NULL) {
              // We found a source address
              *cptr = '\0';
              //diagPrint(F("receiveDataNoAck: 3 '")); diagPrint(data); diagPrintLn('\'');
              if (source) {
                // Store the source
                strncpy(source, data, sourceSize - 1);
              }
              // Next, strip the source address, so that only the data remains.
              strcpy(data, cptr + 1);
              return XRF_OK;
            }
            // Hmm. We get here when the packet only has destination and source, but no data.
            // Weird. Let's treat it as if it was a checksum error.
          }
          status = XRF_CRC_ERROR;
        }
      }
      // Keep on trying
    }
  }
  ++_failedCounter;
  return status;
}

/*
 * Send an Ack to a device (from which we just got a valid packet)
 *
 * The syntax of the whole ACK line is as follows:
 *   <dest> ',' <source> ',' [ <ts> ',' ] "ack" ',' <crc>
 * The timestamp is optional. If the callback getNow holds a valid
 * function pointer we use that to get the current timestamp.
 */
void XRF::sendAck(const char *dest)
{
  if (_getNow) {
    uint32_t ts = (*_getNow)();
    // Assemble a packet with <ts> ',' "ack"
    String str;
    str += ts;
    str += _fldSep;
    str += "ack";
    sendDataNoWait(dest, str.c_str());
  } else {
    // An Ack without a timestamp
    sendDataNoWait(dest, "ack");
  }
}

/*
 * Send a command and wait for a reply
 *
 * This is a convenience function for the application level.
 * Send a packet and expect a packet in return. Both packets must
 * be acked by the XRF send and receive functions.
 *
 * \param dest the destination where to send the packet to
 * \param data the command to send
 * \param reply a buffer to store the reply when/if it is received
 * \param replySize the size of the "reply" buffer
 */
uint8_t XRF::sendDataAndWaitForReply(const char *dest, const char *data, char *reply, size_t replySize)
{
  uint8_t status = XRF_MAXRETRY;

  for (size_t i = 0; i < _nrRetries; ++i) {
    sendData(dest, data);

    status = receiveData(NULL, 0, reply, replySize);
    if (status == XRF_OK) {
      //diagPrint(F("reply2: '")); diagPrint(reply); diagPrintLn('\'');
      break;
    }
  }
  return status;
}

/*
* Set the PAN ID of the XRF device
*
* \return XRF_OK if the operation was successful or else an error code
*/
uint8_t XRF::setPanID(uint16_t panID)
{
  return sendATxSetHexNumber("ATID", panID);
}

/*
 * Get the current PAN ID
 *
 * Valid PAN IDs are 0..0xEFFF. The rest is reserved.
 *
 * \return the PAN ID, or 0xFFFF in case of an error
 */
uint16_t XRF::getPanID()
{
  uint32_t val;
  if (!sendATxGetHexNumber("ATID", &val)) {
    return -1;
  }
  // No validation is done.
  return val;
}

/*
 * Apply Changes, send the ATAC command
 *
 * For the documentation for the ATAC command:
 *  "Returns OK and then applies changes to baud rate, flowcontrol,
 *   radio data rate and radio freq. NOTE: that if you have changed
 *   the baud rate then after the OK message you will need to change
 *   the baudrate at the other end. This does NOT save the configuration
 *   (use ATWR for that)."
 */
uint8_t XRF::doApplyChanges()
{
  return sendCommandWaitForOK("ATAC");
}

/*
* Set the baud rate
*
* This function sets the baud rate of the uart and also the XRF.
* Be careful with this, because if somehow a mismatch is created
* it will be hard to correct it once the XRF configuration
* is made permanent.
*
* \param rate is the baud rate
* \return XRF_OK if the operation was successful or else an error code
*/
uint8_t XRF::setBaudRate(uint32_t rate)
{
  return sendATxSetHexNumber("ATBD", rate);
}

/*
 * Get the current baud rate
 *
 * Common baud rates are: 1200, 2400, 4800, 9600, 31250, 38400, 57600, 115200
 * TODO The above list is from the XRF AT Command Reference. Find out if other
 * baud rates are possible.
 *
 * \return the baud rate
 * \return 0xffffffff (-1) if the operation failed
 */
uint32_t XRF::getBaudRate()
{
  uint32_t val;
  if (!sendATxGetHexNumber("ATBD", &val)) {
    return -1;
  }
  return val;
}

/*
* Set the radio data rate
*
* This function sets the radio data rate.
*
* \param rate is the radio data rate
* \return XRF_OK if the operation was successful or else an error code
*/
uint8_t XRF::setDataRate(uint8_t rate)
{
  return sendATxSetHexNumber("ATDR", rate);
}

/*
 * Get the current radio data rate (1, 2, 3, 4, 5)
 *
 * The AT Command Reference says:
 *   1 = 250k (default)
 *   2 = 38.4k
 *   3 = 1.2k
 *   4 = 100k
 *   5 = 50k
 *
 * \return the radio data rate (1, 2, 3, 4, 5)
 * \return 0 this indicates an error
 */
uint8_t XRF::getDataRate()
{
  uint32_t val;
  if (!sendATxGetHexNumber("ATDR", &val)) {
    return 0;
  }
  return val;
}

/*
* Set the packet size
*
* This function sets the maximum radio packet data length
*
* From the documentation:
*   "minimum value is 1 maximum is F0 (decimal 240)
*    Default 0C (12 data bytes)
*    Note:  The XRF will not receive packets that are longer than this
*    setting, so it needs to be set on all connected XRFs.
*    Note: If you have a much larger packet size than you expect to send
*    to the node then you will find that spurious packets are more frequent
*    and therefore more packets will be dropped."
*
* \param size is maximum radio packet data length
* \return XRF_OK if the operation was successful or else an error code
*/
uint8_t XRF::setPacketSize(uint8_t size)
{
  return sendATxSetHexNumber("ATPK", size);
}

/*
 * Get the current maximum radio packet data length
 *
 * \return the
 * \return 0 this indicates an error
 */
uint8_t XRF::getPacketSize()
{
  uint32_t val;
  if (!sendATxGetHexNumber("ATPK", &val)) {
    return 0;
  }
  return val;
}

/*
* Set the Packet timeout
*
* This function sets the Packet timeout
*
* From the documentation:
*   "The time in milliseconds before a packet is sent if packet is not
*    complete (hex)
*    Range 1 to FFFF (65535)
*    Default is 10 (16mS) - from XRF/ERF firmware 0.41, URF firmware 0.20
*    onwards, previous default was 64 (100 mS)"
*
* \param size is the Packet timeout
* \return XRF_OK if the operation was successful or else an error code
*/
uint8_t XRF::setPacketTimeout(uint16_t timeout)
{
  return sendATxSetHexNumber("ATRO", timeout);
}

/*
 * Get the current maximum radio packet data length
 *
 * \return the
 * \return 0 this indicates an error
 */
uint16_t XRF::getPacketTimeout()
{
  uint32_t val;
  if (!sendATxGetHexNumber("ATRO", &val)) {
    return 0;
  }
  return val;
}

/*
* Set the sleep of the XRF and record the pin to bring the device to sleep
*
* Part of the AT documentation goes like this:
*   0 – no sleep, the /SLEEP pin has no effect
*   1 – when the /SLEEP pin is set high or un-connected the XRF will run,
*       when the sleep pin is set low the XRF will sleep (power consumption
*       when sleeping of around 150uA)
*   2 – when the /SLEEP pin is set low the XRF will run, when the sleep pin
*       is un-connected or set high the XRF will sleep. This is the sleep
*       mode with the lowest sleeping power consumption (<0.5uA)
*
* \param mode the parameter for the ATSM
* \return XRF_OK if the operation was successful or else an error code
*/
uint8_t XRF::setSleepMode(uint8_t mode, uint8_t sleepPin)
{
  uint8_t status = XRF_NOT_IMPLEMENTED;

  diagPrint(F("XRF::setSleepMode mode ")); diagPrintLn(mode);
  diagPrint(F("XRF::setSleepMode sleepPin ")); diagPrintLn(sleepPin);
  _sleepPin = sleepPin;
  switch (mode) {
  case 1:
  case 2:
    pinMode(_sleepPin, OUTPUT);
    break;
  case 0:
  default:
    break;
  }

  // Get the current ATSM mode
  // Side effect is that if it succeeds, it is woken up.
  _sleepMode = getATSM();

  switch (mode) {
  case 0:
  case 1:
  case 2:
    if (_sleepMode != mode) {
      _sleepMode = mode;
      status = sendATxSetHexNumber("ATSM", _sleepMode);
      if (status != XRF_OK) {
        return status;
      }
    } else {
      status = XRF_OK;
    }
    break;
  default:
    _sleepMode = 0xFF;
    break;
  }

  // Change the pin to the wake up state.
  switch (_sleepMode) {
  case 0:
    // No-op
    break;
  case 1:
    // Set pin HIGH to not sleep
    digitalWrite(_sleepPin, HIGH);
    _sleepPinStatus = HIGH;
    break;
  case 2:
    // Set pin LOW to not sleep
    digitalWrite(_sleepPin, LOW);
    _sleepPinStatus = LOW;
    break;
  default:
    break;
  }
  return status;
}

/*
 * Get the current ATSM
 *
 * The difficulty is that we don't know in which mode
 * the XRF currently is. If it is already mode 1 or 2
 * then we simply have to try the sleep pin on and off
 * to make it wake up.
 */
uint8_t XRF::getATSM()
{
  uint8_t mode;
  /*
   */
  // Assume it is in mode 0, it should respond right away
  diagPrint(F("XRF::setSleepMode _sleepPin ")); diagPrintLn(_sleepPin);
  mode = _getATSM();
  diagPrint(F("XRF::getATSM mode0 ")); diagPrintLn(mode);
  if (mode == 0xFF) {
    // We now must have a pin. If not, we can't do anything else
    if (_sleepPin) {
      // Assume it is in mode 1. Set the pin to wake up mode 1.
      digitalWrite(_sleepPin, HIGH);
      _sleepPinStatus = HIGH;
      mode = _getATSM();
      diagPrint(F("XRF::getATSM mode1 ")); diagPrintLn(mode);
      if (mode == 0xFF) {
        // Assume it is in mode 2. Set the pin to wake up mode 2.
        digitalWrite(_sleepPin, LOW);
        _sleepPinStatus = LOW;
        mode = _getATSM();
        diagPrint(F("XRF::getATSM mode2 ")); diagPrintLn(mode);
        if (mode == 0xFF) {
          // We're out of suggestions. Perhaps the caller can retry.
          return 0xFF;
        }
      }
    }
  }
  return mode;
}

/*
 * Get the current ATSM
 */
uint8_t XRF::_getATSM()
{
  uint32_t val;
  if (!sendATxGetHexNumber("ATSM", &val)) {
    return 0xff;
  }
  return val;
}

void XRF::sleep()
{
  uint8_t newStatus = 0xFF;
  switch (_sleepMode) {
  case 0:
    // No-op
    break;
  case 1:
    newStatus = LOW;
    break;
  case 2:
    newStatus = HIGH;
    break;
  default:
    break;
  }
  setSleepPin(newStatus, true);
}

void XRF::wakeUp()
{
  uint8_t newStatus = 0xFF;
  switch (_sleepMode) {
  case 0:
    // No-op
    break;
  case 1:
    newStatus = HIGH;
    break;
  case 2:
    newStatus = LOW;
    break;
  default:
    break;
  }
  setSleepPin(newStatus, false);
}

void XRF::setSleepPin(uint8_t newStatus, bool isSleep)
{
  if (newStatus != 0xFF && _sleepPinStatus != newStatus) {
    //diagPrint(isSleep ? F("go to sleep") : F("wake up"));
    //diagPrint(F(", pin setting ")); diagPrintLn(newStatus == LOW ? F("LOW") : F("HIGH"));
    if (isSleep) {
      // Allow a bit of time to write whatever is in the output buffer
      delay(_sleepDelay);
    }
    // Set pin to sleep
    digitalWrite(_sleepPin, newStatus);
    _sleepPinStatus = newStatus;
  }
}

//////////////////////////
// Private methods
//////////////////////////

void XRF::flushInput()
{
  int c;
  while ((c = _myStream->read()) >= 0) {
    //diagPrintLn(c, HEX);
  }
}

uint8_t XRF::waitForOK(uint16_t timeout)
{
  char buffer[10];
  if (!readLine(buffer, sizeof(buffer), timeout)) {
    return XRF_TIMEOUT;
  }
  return strcmp(buffer, "OK") == 0 ? XRF_OK : XRF_NOT_OK;
}

/*
 * Read a non-empty line from the input
 *
 * \param buffer pointer to store the result
 * \param size how many bytes can be stored in the result (including \0)
 * \param timeout
 */
bool XRF::readLine(char *buffer, size_t size, uint16_t timeout)
{
  size_t len;
  bool seenCR = false;
  bool doneTooLong = false;
  uint32_t ts_waitLF = 0;
  int c;

  len = 0;
  uint32_t ts_max = millis() + timeout;
  while (!isTimedOut(ts_max)) {
    if (seenCR) {
      c = _myStream->peek();
      // ts_waitLF is guaranteed to be non-zero
      if ((c == -1 && isTimedOut(ts_waitLF)) || (c != -1 && c != '\n')) {
        //diagPrint(F("readLine:  peek '")); diagPrint(c); diagPrintLn('\'');
        // Line ended with just <CR>. That's OK too.
        goto ok;
      }
      // Only \n should fall through
    }

    c = _myStream->read();
    if (c < 0) {
      continue;
    }
    seenCR = c == '\r';
    if (c == '\r') {
      ts_waitLF = millis() + 50;       // Wait a short while for an optional LF
    } else if (c == '\n') {
      if (len > 0) {
        goto ok;
      }
      // An empty line. Continue to wait.
    } else {
      if (len < size - 1) {
        buffer[len++] = c;
      } else {
        if (!doneTooLong) {
          buffer[len] = '\0';
          diagPrint(F("readLine:  line too long '")); diagPrint(buffer); diagPrintLn('\'');
          doneTooLong = true;
        }
      }
    }
  }
  //diagPrintLn(F("readLine timed out"));
  return false;         // This indicates: timed out

ok:
  buffer[len] = '\0';
  //diagPrint(F("readLine: '")); diagPrint(buffer); diagPrintLn('\'');
  return true;
}

/*
 * Enter command mode
 *
 * This mode is entered by sending +++ to the device. If it replies
 * with OK, then we're in command mode. After 5 seconds timeout the
 * unit automatically leaves command mode.
 * It is also possible to send ATDN to leave the command mode right
 * away.
 *
* \return XRF_OK if the operation was successful or else an error code
 */
uint8_t XRF::enterCmndMode()
{
  uint8_t status;

  if (_inCmndMode) {
    // Check if we are really in Command Mode
    sendCommand("AT");
    status = waitForOK();
    if (status == XRF_OK) {
      return status;
    }
    _inCmndMode = false;
  }

  for (size_t i = 0; !_inCmndMode && i < _nrRetries; ++i) {
    diagPrintLn(F(">> +++"));
    // delay 1 second
    delay(1000);
    _myStream->print("+++");
    delay(500);           // This could be up to 1000, but then we could miss the OK
    // Flush any chars that were received in the meantime
    flushInput();

    // wait until it replies with "OK"
    status = waitForOK();
    _inCmndMode = status == XRF_OK;
  }
  return _inCmndMode ? XRF_OK : XRF_TIMEOUT;
}

uint8_t XRF::leaveCmndMode()
{
  if (!_inCmndMode) {
    return XRF_NOT_IN_CMDMODE;
  }

  sendCommand("ATDN");

  // wait until it replies with "OK"
  uint8_t status = waitForOK();
  _inCmndMode = false;          // No matter what we get, we're out of Command Mode
  return status;
}

void XRF::sendCommand(const char *cmd)
{
  flushInput();
  diagPrint(F(">> ")); diagPrintLn(cmd);
  _myStream->print(cmd);
  _myStream->print('\r');
}

uint8_t XRF::sendCommandWaitForOK(const char *cmd, uint16_t timeout)
{
  sendCommand(cmd);
  return waitForOK(timeout);
}

/*
 * Send ATcc command and read the value
 *
 * \param at the ATcc command
 * \param num pointer to store the result
 */
bool XRF::sendATxGetHexNumber(const char *at, uint32_t *num)
{
  uint8_t status;
  char buffer[10];

  status = enterCmndMode();
  if (status != XRF_OK) {
    return false;
  }

  sendCommand(at);
  // Wait for:
  //    5AA5<CR>
  //    OK<CR>
  if (!readLine(buffer, sizeof(buffer))) {
    // Timed out
    diagPrintLn(F("sendATxGetHexNumber: timed out"));
    return false;
  }
  if ((status = waitForOK()) != XRF_OK ) {
    // Missing OK
    diagPrint(F("sendATxGetHexNumber, missing OK ")); diagPrintLn(status);
    return false;
  }
  uint32_t val;
  if (!getUValue(buffer, &val, 16)) {
    // Invalid hex number
    diagPrintLn(F("sendATxGetHexNumber: invalid number"));
    return false;
  }

  //leaveCmndMode();      // ignore the result of this

  if (num) {
    *num = val;
  }
  return true;
}

/*
 * Send the AT command plus the new value
 *
 * \param at the ATcc command
 * \param num pointer to store the result
 */
uint8_t XRF::sendATxSetHexNumber(const char *at, uint32_t num)
{
  uint8_t status;
  char buffer[15];      // Should be big enough for "AT0hhhhhhhh"
  char *ptr;

  status = enterCmndMode();
  if (status != XRF_OK) {
    return status;
  }

  strcpy(buffer, at);
  strcat(buffer, "0");
  ptr = buffer + strlen(buffer);
  ultoa(num, ptr, 16);
  sendCommand(buffer);
  if ((status = waitForOK()) != XRF_OK ) {
    // Missing OK
    diagPrint(F("sendATxSetHexNumber, missing OK: ")); diagPrintLn(status);
  }
  return status;
}

/*
 * Find the CRC in the line of text
 *
 * First step is to find the last comma. And if it is
 * found return the character after it.
 * The help stripping the CRC later we also return the pointer
 * to the comma before the checksum.
 *
 * \param txt a text string
 * \param crc pointer for the result
 * \param cptr pointer to where CRC starts (the comma before the checksum)
 * \return true if a CRC was found
 */
bool XRF::findCrc(char *txt, uint16_t *crc, char **cptr)
{
  char *ptr;
  ptr = txt + strlen(txt);
  while (ptr > txt) {
    *cptr = ptr - 1;
    if (**cptr == _fldSep) {
      uint32_t val;
      if (getUValue(ptr, &val)) {
        *crc = val;
        return true;
      }
      break;
    }
    --ptr;
  }

  return false;
}

/*
 * \brief Compute CRC16 of a byte buffer (CCITT)
 */
uint16_t XRF::crc16_ccitt(uint8_t * buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc = _crc_ccitt_update(crc, *buf++);
    }
    return crc;
}

/*
 * \brief Compute CRC16 of a byte buffer (XMODEM)
 */
uint16_t XRF::crc16_xmodem(uint8_t * buf, size_t len)
{
    uint16_t crc = 0;
    while (len--) {
        crc = _crc_xmodem_update(crc, *buf++);
    }
    return crc;
}

bool XRF::getUValue(const char *buffer, uint32_t * value, int base)
{
  char *eptr;
  *value = strtoul(buffer, &eptr, base);
  if (eptr != buffer) {
    return true;
  }
  return false;
}
