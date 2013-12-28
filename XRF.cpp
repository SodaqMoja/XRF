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
#include <util/crc16.h>
#include <Arduino.h>
#include "XRF.h"

XRF::XRF()
{
  _myStream = 0;
  _eol = '\n';
  _diagStream = 0;
  _panID = 0x5AA5;
  _nrRetries = 3;
  _retryTimeout = 2000;
  _inCmndMode = false;
  _sleepMode = 0;
  _sleepPin = 0;
}

/*
* Initialize XRF
*
* \param uart an instance of Stream for the connected XRF module
* \param nrRetries how often to try to send a data packet when no ACK is received
* \param retryTimeout number of milliseconds before retrying
* \return nothing
*/
void XRF::init(Stream &stream,
                uint8_t nrRetries, uint16_t retryTimeout)
{
  _myStream = &stream;
}

/*
* Send ASCII data via the XRF device
*
* The data is sent as ASCII. The CRC is appended and an end-of-line
* is added too.
*
* \param data is a pointer to the ASCII data
*/
void XRF::sendData(const char *data)
{
  diagPrint(F("SendData: '")); diagPrint(data); diagPrintLn('\'');

  // All current input can be flushed. It is not for us.
  flushInput();

  uint16_t crc = crc16_xmodem((uint8_t *)data, strlen(data));
  _myStream->print(data);
  _myStream->print(',');
  _myStream->print(crc);
  _myStream->print(_eol);
}

/*
 * Wait for a line of text that starts with the prefix
 *
 * \param prefix the line that we want starts with this
 * \param data a pointer to store the result
 * \param size the maximum number of bytes to store in the result buffer (including \0)
 * \param timeout the maximum number of milliseconds for the whole operation
 * \return XRF_OK if the operation was successful or else an error code
 */
uint8_t XRF::receiveData(const char *prefix, char *data, size_t size, uint16_t timeout)
{
  //diagPrint(F("receiveData prefix: '")); diagPrint(prefix); diagPrintLn('\'');
  uint32_t ts_max = millis() + timeout;
  while (!isTimedOut(ts_max)) {
    if (readLine(data, size)) {
      // Does the prefix match?
      if (strncmp(data, prefix, strlen(prefix)) == 0) {
        // Yes, it does.
        // TODO Verify checksum
        uint16_t crc;
        char *cptr;
        if (findCrc(data, &crc, &cptr)) {
          //diagPrint(F("receiveData crc: ")); diagPrintLn(crc);
          // Strip the checksum
          *cptr = '\0';
          uint16_t crc1 = crc16_xmodem((uint8_t *)data, strlen(data));
          //diagPrint(F("receiveData: '")); diagPrint(data); diagPrintLn('\'');
          //diagPrint(F("receiveData checksum : ")); diagPrintLn(crc == crc1 ? "OK" : "not OK");
          return crc1 == crc ? XRF_OK : XRF_CRC_ERROR;
        }
      }
      // Keep on trying
    }
  }
  return XRF_TIMEOUT;
}

/*
 * Wait for a line of text with a checksum
 *
 * First step is to simply receive a line. Then it tries
 * to find a checksum at the end of the line. The checksum
 * is a comma and a number. The number is the checksum of
 * all the characters in the line upto (not including the
 * comma and the number)
 *
 * \param data a pointer to store the result
 * \param size the maximum number of bytes to store in the result buffer (including \0)
 * \param timeout the maximum number of milliseconds for the whole operation
 * \return XRF_OK if the operation was successful or else an error code
 */
uint8_t XRF::receiveData(char *data, size_t size, uint16_t timeout)
{
  // Read a line
  // TODO Check and strip CRC
  return readLine(data, size, timeout) ? XRF_OK : XRF_TIMEOUT;
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
  // First set the pin so that we don't go to sleep right away
  _sleepPin = sleepPin;
  switch (mode) {
  case 0:
    // No-op
    break;
  case 1:
    // Set pin HIGH to not sleep
    digitalWrite(_sleepPin, HIGH);
    break;
  case 2:
    // Set pin LOW to not sleep
    digitalWrite(_sleepPin, LOW);
    break;
  default:
    break;
  }

  switch (mode) {
  case 0:
  case 1:
  case 2:
    _sleepMode = mode;
    status = sendATxSetHexNumber("ATSM", _sleepMode);
    break;
  default:
    _sleepMode = 0;
    break;
  }
  return status;
}

void XRF::sleep()
{
  switch (_sleepMode) {
  case 0:
    // No-op
    break;
  case 1:
    // Set pin LOW to sleep
    digitalWrite(_sleepPin, LOW);
    break;
  case 2:
    // Set pin HIGH to sleep
    digitalWrite(_sleepPin, HIGH);
    break;
  default:
    break;
  }
}

void XRF::wakeUp()
{
  switch (_sleepMode) {
  case 0:
    // No-op
    break;
  case 1:
    // Set pin HIGH to wake up
    digitalWrite(_sleepPin, HIGH);
    break;
  case 2:
    // Set pin LOW to wake up
    digitalWrite(_sleepPin, LOW);
    break;
  default:
    break;
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
  uint32_t ts_waitLF = 0;
  int c;

  len = 0;
  uint32_t ts_max = millis() + timeout;
  while (!isTimedOut(ts_max)) {
    if (seenCR) {
      c = _myStream->peek();
      if (c != '\n' || isTimedOut(ts_waitLF)) {
        // Line ended with just <CR>. That's OK too.
        goto ok;
      }
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
      }
    }
  }
  diagPrintLn(F("readLine timed out"));
  return false;         // This indicates: timed out

ok:
  buffer[len++] = '\0';
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
  return status;
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
  char *eptr;

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
  uint32_t val = strtoul(buffer, &eptr, 16);
  if (eptr == buffer) {
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
    return false;
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
    if (**cptr == ',') {
      char *eptr;
      *crc = strtoul(ptr, &eptr, 0);
      if (eptr != ptr) {
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
