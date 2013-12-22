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

#include <Arduino.h>
#include "XRF.h"

/*
* Initialize XRF
*
* \param devID the identifier used in data packets
* \param uart an instance of Stream for the connected XRF module
* \param nrRetries how often to try to send a data packet when no ACK is received
* \param retryTimeout number of milliseconds before retrying
* \return nothing
*/
void XRF::init(uint16_t devID, Stream &stream,
                uint8_t nrRetries, uint16_t retryTimeout)
{
  _myStream = &stream;
  _devID = devID;
}

/*
* Set the PAN ID of the XRF device
*
* \return 0 if the operation was successful or else an error code
*/
uint8_t XRF::setPanID(uint16_t panID)
{
  return XRF_UNKNOWN;
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
  uint8_t status;
  char buffer[10];
  char *eptr;

  status = enterCmndMode();
  if (status != XRF_OK) {
    return 0xffff;
  }

  sendCommand("ATID");
  // Wait for:
  //    5AA5<CR>
  //    OK<CR>
  if (!readLine(buffer, sizeof(buffer))) {
    // Timed out
    return 0xffff;
  }
  if (!waitForOK()) {
    // Missing OK
    return 0xffff;
  }
  uint16_t retval = strtoul(buffer, &eptr, 16);
  if (eptr == buffer) {
    // Invalid hex number
    return 0xffff;
  }

  leaveCmndMode();      // ignore the result of this
  return retval;
}

/*
* Set the baud rate
*
* This function sets the baudrate of the uart and also the XRF.
* Be careful with this, because if somehow a mismatch is created
* it will be hard to correct it once the XRF configuration
* is made permanent.
*
* \param rate is the baudrate
* \return XRF_OK if the operation was successful or else an error code
*/
uint8_t XRF::setBaudRate(uint32_t rate)
{
  return XRF_UNKNOWN;
}

uint8_t XRF::setATBD(uint16_t rate)
{
  return XRF_UNKNOWN;
}

/*
 * Get the current baud rate
 *
 * Common baud rates are: 1200, 2400, 4800, 9600, 31250, 38400, 57600, 115200
 * TODO The above list is from the XRF AT Command Reference. Find out if other
 * baud rates are possible.
 *
 * \return the baudrate
 */
uint32_t XRF::getBaudRate()
{
  uint8_t status;
  char buffer[10];
  char *eptr;

  status = enterCmndMode();
  if (status != XRF_OK) {
    return 0xffff;
  }

  sendCommand("ATBD");
  // Wait for:
  //    02580<CR>
  //    OK<CR>
  if (!readLine(buffer, sizeof(buffer))) {
    // Timed out
    return 0xffff;
  }
  if (!waitForOK()) {
    // Missing OK
    return 0xffff;
  }
  uint16_t retval = strtoul(buffer, &eptr, 16);
  if (eptr == buffer) {
    // Invalid hex number
    return 0xffff;
  }

  leaveCmndMode();      // ignore the result of this
  return retval;
}

//////////////////////////
// Private methods
//////////////////////////

void XRF::flushInput()
{
  int c;
  while ((c = _myStream->read()) >= 0) {
    //diagPrint((char)c);
  }
}

bool XRF::waitForOK(uint16_t timeout)
{
  char buffer[10];
  if (!readLine(buffer, sizeof(buffer), timeout)) {
    return false;
  }
  return strcmp(buffer, "OK") == 0;
}

/*
 * Read a line from the input
 *
 * This function should only be called when in command mode. Otherwise
 * it doesn't make sense, because the data on the air is most likely
 * just binary.
 */
bool XRF::readLine(char *buffer, size_t size, uint16_t timeout)
{
  size_t len;

  len = 0;
  uint32_t ts_max = millis() + timeout;
  while (!isTimedOut(ts_max)) {
    int c = _myStream->read();
    if (c < 0) {
      continue;
    }
    if (c == '\r') {
      if (len > 0) {
        goto ok;
      }
      // An empty line. Continue to wait.
    } else if (c == '\n') {
      // Skip LF.
      // We're expecting everything to be terminated with just a CR
    } else {
      if (len < size - 1) {
        buffer[len++] = c;
      }
    }
  }
  return false;         // This indicates: timed out

ok:
  buffer[len++] = '\0';
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
  // delay 1 second
  delay(1000);
  _myStream->print("+++");
  delay(500);           // This could be up to 1000, but then we could miss the OK
  // Flush any chars that were received in the meantime
  flushInput();

  // wait until it replies with "OK"
  return waitForOK() ? XRF_OK : XRF_UNKNOWN;
}

uint8_t XRF::leaveCmndMode()
{
  sendCommand("ATDN");
  // wait until it replies with "OK"
  return waitForOK() ? XRF_OK : XRF_UNKNOWN;
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
