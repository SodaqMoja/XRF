#!/usr/bin/env python

import sys
import serial
import time

if sys.version_info >= (3, 0):
    def cnvToCharBuf(b):
        return b.decode('latin1')
else:
    def cnvToCharBuf(b):
        return b

class XRF(serial.Serial):
    'XRF is a class to read and write to a XRF device via serial'

    XRF_OK = 0
    XRF_ERR = 1
    XRF_NOT_OK = 2              # Failed to see OK
    XRF_NACK = 3
    XRF_TIMEOUT = 4
    XRF_MAXRETRY = 5
    XRF_CRC_ERROR = 6
    XRF_FAIL_CMDMODE = 7        # Failed to enter command mode (+++)
    XRF_NOT_IN_CMDMODE = 8      # Attempting a command while not in Command Mode
    XRF_NOT_IMPLEMENTED = 9     # Function not yet implemented
    XRF_UNKNOWN = 10

    def __init__(self, *args, **kwargs):
        # timeout=0 - non blocking mode
        # timeout=0.01 - timeout after 10 milliseconds
        if 'timeout' not in kwargs:
            kwargs['timeout'] = 0.01
        super(XRF, self).__init__(*args, **kwargs)
        self._panID = 0
        self._nrRetries = 3
        self._retryTimeout = 500
        self._fldSep = ','
        self._eol = '\n'
        self._inCmndMode = False
        self._sleepMode = None
        self._peekChar = None
        self._getNow = None
        self.flushInput()

    def init(self, name, panID):
        self._devName = name
        self._panID = panID
        while self._panID != 0 and self.setPanID(self._panID) != self.XRF_OK:
            # Wait a longer while and hope for the best when retrying
            self._delay(5000)
        self._leaveCmndMode()

    @property
    def getNow(self):
        return self._getNow
    @getNow.setter
    def getNow(self, func):
        self._getNow = func

    def sendData(self, dest, data):
        '''
        * Send ASCII data via the XRF device
        *
        * The data is sent as ASCII. The CRC is appended and an end-of-line
        * is added too.
        *
        * \param dest the destination
        * \param data the ASCII data
        '''
        # Flush all input so that we are not disturbed when looking for an ack
        self.flushInput()
        status = self.XRF_UNKNOWN
        for i in range(self._nrRetries):
            self._sendDataNoWait(dest, data)
            # Wait for an ack
            status = self._waitForAck(dest, self._retryTimeout)
            if status in (self.XRF_OK, self.XRF_NACK):
                break
            status = self.XRF_MAXRETRY
        return status

    def _sendDataNoWait(self, dest, data):
        '''
        * Send ASCII data via the XRF device
        *
        * The data is sent as ASCII. The CRC is appended and an end-of-line
        * is added too.
        *
        * \param dest the destination
        * \param data the ASCII data
        '''
        line = self._fldSep.join([dest, self._devName, data])
        crc = self.get_crc16_xmodem(line)
        line = self._fldSep.join([line, str(crc)])
        self.write(line + self._eol)

    def _waitForAck(self, expectSource, timeout):
        '''
         * Wait for an "ack"
         *
         * The syntax of the ack message (stripped <dest> and <crc>) is
         *   <source> ',' "ack"
        '''
        for i in range(self._nrRetries):
            status, source, data = self._receiveDataNoAck(self._devName, timeout)
            if status == self.XRF_OK:
                if data == 'ack':
                    return self.XRF_OK
                if data == 'nack':
                    return self.XRF_NACK
            status = self.XRF_MAXRETRY
        return status

    def receiveData(self, timeout):
        '''
        * Wait for a packet that has our name as destination.
        '''
        status, source, data = self._receiveDataNoAck(self._devName, timeout)
        if status == self.XRF_OK:
            # Send an ACK
            self._sendAck(source)
        return status, source, data

    def _receiveDataNoAck(self, expectDest, timeout):
        '''
        * Wait for a line of text that starts with the prefix
        *
        * \param expectDest the destination name that the packet must have
        * \param timeout the maximum number of milliseconds for the whole operation
        * \return status, source, data
        * \return XRF_OK if the operation was successful or else an error code
        '''
        tsMax = self._millis() + timeout
        readTimeout = max(timeout, 1000)
        while not self._timedOut(tsMax):
            line = self._readLine(readTimeout)
            if line is None or len(line) == 0:
                continue
            # The packet must at least have 3 fields
            try:
                dest, source, data = line.split(self._fldSep, 2)
                #self._diagPrintLn("_receiveDataNoAck: dest=%(dest)s data='%(data)s'" % vars())
            except ValueError:
                # Keep on trying
                continue
            # Is this a packet for us?
            if dest != expectDest:
                #self._diagPrintLn("_receiveDataNoAck: dest=%(dest)s expectDest='%(expectDest)s'" % vars())
                continue
            # Does it have a valid CRC?
            line, crc = self._findCrc(line)
            if crc is None:
                continue
            try:
                crc = int(crc)
            except ValueError:
                self._diagPrintLn("_receiveDataNoAck: line=%(line)s crc='%(crc)s'" % vars())
                continue
            crc1 = self.get_crc16_xmodem(line)
            if crc == crc1:
                # Yes
                # Strip destination address (that's us)
                try:
                    dest, source, data = line.split(self._fldSep, 2)
                except:
                    # Not enough fields
                    return self.XRF_UNKNOWN, None, None
                #self._diagPrintLn(" _receiveDataNoAck: source=%(source)s data='%(data)s'" % vars())
                return self.XRF_OK, source, data
            # No, keep on trying
            #self._diagPrintLn("_receiveDataNoAck: crc=%(crc)d != crc1=%(crc1)d" % vars())

        return self.XRF_TIMEOUT, None, None

    def _sendAck(self, dest):
        if self._getNow is not None:
            ts = self._getNow()
            self._sendDataNoWait(dest, self._fldSep.join([str(ts), "ack"]))
        else:
            self._sendDataNoWait(dest, "ack")

    def sendDataAndWaitForReply(self, dest, data):
        '''
        * Send a command and wait for a reply
        *
        * This is a convenience function for the application level.
        * Send a packet and expect a packet in return. Both packets must
        * be acked by the XRF send and receive functions.
        *
        * \param dest the destination where to send the packet to
        * \param data the bytes to send
        '''
        status = self.XRF_UNKNOWN
        reply = None
        for i in range(self._nrRetries):
            self._sendData(dest, data)
            status, reply = self._receiveData(self._retryTimeout)
            if status == self.XRF_OK:
                break
        return status, reply

    def setPanID(self, panID):
        'Set the PAN ID of the XRF device'
        return self._sendATxSetHexNumber("ATID", panID)

    def getPanID(self):
        'Get the current PAN ID'
        status, val = self._sendATxGetHexNumber("ATID")
        if not status:
            return -1
        return val

    def doApplyChanges(self):
        '''
        * Apply Changes, send the ATAC command
        *
        * For the documentation for the ATAC command:
        *  "Returns OK and then applies changes to baud rate, flowcontrol,
        *   radio data rate and radio freq. NOTE: that if you have changed
        *   the baud rate then after the OK message you will need to change
        *   the baudrate at the other end. This does NOT save the configuration
        *   (use ATWR for that)."
        '''
        return self._sendCommandWaitForOK("ATAC")

    def setBaudRate(self, rate):
        '''
        * Set the baud rate
        *
        * This function sets the baud rate of the uart and also the XRF.
        * Be careful with this, because if somehow a mismatch is created
        * it will be hard to correct it once the XRF configuration
        * is made permanent.
        *
        * \param rate is the baud rate
        * \return XRF_OK if the operation was successful or else an error code
        '''
        return self._sendATxSetHexNumber("ATBD", rate)

    def getBaudRate(self):
        '''
         * Get the current baud rate
         *
         * Common baud rates are: 1200, 2400, 4800, 9600, 31250, 38400, 57600, 115200
         * TODO The above list is from the XRF AT Command Reference. Find out if other
         * baud rates are possible.
         *
         * \return the baud rate
         * \return 0xffffffff (-1) if the operation failed
        '''
        status, val = self._sendATxGetHexNumber("ATBD")
        if not status:
            val = -1
        return val

    def setDataRate(self, rate):
        '''
        * Set the radio data rate
        *
        * This function sets the radio data rate.
        *
        * \param rate is the radio data rate
        * \return XRF_OK if the operation was successful or else an error code
        '''
        return self._sendATxSetHexNumber("ATDR", rate)

    def getDataRate(self):
        '''
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
        '''
        status, val = self._sendATxGetHexNumber("ATDR")
        if not status:
            val = 0
        return val

    def setPacketSize(self, size):
        '''
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
        '''
        return self._sendATxSetHexNumber("ATPK", size)

    def getPacketSize(self):
        '''
         * Get the current maximum radio packet data length
         *
         * \return the
         * \return 0 this indicates an error
        '''
        status, val = self._sendATxGetHexNumber("ATPK")
        if not status:
            val = 0
        return val

    def setPacketTimeout(self, timeout):
        '''
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
        '''
        return self._sendATxSetHexNumber("ATRO", timeout)

    def getPacketTimeout(self):
        '''
         * Get the current maximum radio packet data length
         *
         * \return the
         * \return 0 this indicates an error
        '''
        status, val = self._sendATxGetHexNumber("ATRO")
        if not status:
            val = 0
        return val

    def _waitForOK(self, timeout=500):
        line = self._readLine(timeout)
        if line is None:
            return self.XRF_TIMEOUT
        if line == "OK":
            return self.XRF_OK
        if line == "ERR":
            return self.XRF_ERR
        return self.XRF_NOT_OK

    def _peek(self):
        if self._peekChar is None:
            if self.inWaiting() > 0:
                #self._diagPrintLn("_peek - inWaiting %d" % self.inWaiting())
                self._peekChar = cnvToCharBuf(self.read(1))[0]
        #self._diagPrintLn("_peek - '%s'" % repr(self._peekChar))
        return self._peekChar

    def _readChar(self):
        if self._peekChar is not None:
            c = self._peekChar
            self._peekChar = None
        else:
            c = cnvToCharBuf(self.read(1))[0]
        #self._diagPrintLn("_readChar %s" % repr(c))
        return c

    def _readLine(self, timeout=500):
        'Read a non-empty line from the input'
        #self._diagPrintLn("_readLine - start")
        seenCR = False
        tsWaitLF = 0
        tsMax = self._millis() + timeout
        line = ''
        while not self._timedOut(tsMax):
            c = None
            if seenCR:
                if self._peek() is None and self._timedOut(tsWaitLF):
                    # No LF after a CR, return the line (can be empty)
                    #self._diagPrintLn("_readLine - end1 '%(line)s'" % vars())
                    return line
                if self._peek() == '\n':
                    # Eat the LF
                    c = self._readChar()
                    # No LF after a CR, return the line (can be empty)
                    #self._diagPrintLn("_readLine - end2 '%(line)s'" % vars())
                    return line
            if self._peek() is None:
                # TODO Find out if we need a short delay like this
                self._delay(100)
                continue

            c = self._readChar()
            seenCR = c == '\r'
            if seenCR:
                tsWaitLF = self._millis() + 50     # Wait a short while for an optional LF
            elif c == '\n':
                # Return the line (can be empty)
                #self._diagPrintLn("_readLine - end3 '%(line)s'" % vars())
                return line
            else:
                line += c
        # If we get here there was a timeout
        #self._diagPrintLn("_readLine - timedout")
        return None

    def _enterCmndMode(self):
        '''
        * Enter command mode
        *
        * This mode is entered by sending +++ to the device. If it replies
        * with OK, then we're in command mode. After 5 seconds timeout the
        * unit automatically leaves command mode.
        * It is also possible to send ATDN to leave the command mode right
        * away.
        *
        * \return XRF_OK if the operation was successful or else an error code
        '''
        if self._inCmndMode:
            # Check if we are really in Command Mode
            self._sendCommand("AT")
            status = self._waitForOK()
            if status == self.XRF_OK:
                return status
            self._inCmndMode = False

        for i in range(self._nrRetries):
            self._diagPrintLn(">> +++")
            self._delay(1000)
            self.write("+++")
            self._delay(500)           # This could be up to 1000, but then we could miss the OK
            # Flush any chars that were received in the meantime
            #self.flushInput()

            # wait until it replies with "OK"
            status = self._waitForOK(1000);
            self._inCmndMode = status == self.XRF_OK or status == self.XRF_ERR
            self._diagPrintLn("_enterCmndMode - status=%d" % status)
            if self._inCmndMode:
                break
        if self._inCmndMode:
            return self.XRF_OK
        return self.XRF_TIMEOUT

    def _leaveCmndMode(self):
        if not self._inCmndMode:
            return self.XRF_NOT_IN_CMDMODE
        self._sendCommand("ATDN")

        # wait until it replies with "OK"
        status = self._waitForOK()
        self._inCmndMode = False;          # No matter what we get, we're out of Command Mode
        return status

    def _sendCommand(self, cmd):
        self.flushInput()
        self._diagPrintLn(">> " + cmd)
        self.write(cmd)
        self.write('\r')

    def _sendCommandWaitForOK(self, cmd, timeout=2000):
        self._sendCommand(cmd)
        return self._waitForOK(timeout)

    def _sendATxGetHexNumber(self, at):
        'Send ATcc command and read the value'
        status = self._enterCmndMode()
        if status != self.XRF_OK:
            return False, 0

        self._sendCommand(at)
        #Wait for:
        #    5AA5<CR>
        #    OK<CR>
        line = self._readLine()
        if line is None:
            # Timed out
            self._diagPrintLn("sendATxGetHexNumber: timed out")
            return False, 0
        status = self._waitForOK()
        if status != self.XRF_OK:
            # Missing OK
            self._diagPrintLn("sendATxGetHexNumber, missing OK: %d" % status)
            return False, 0
        try:
            val = int(line, 16)
        except:
            self._diagPrintLn("sendATxGetHexNumber: invalid number: " + line)
            return False, 0

        self._leaveCmndMode()       # ignore the result of this

        return True, val

    def _sendATxSetHexNumber(self, at, num):
        'Send the AT command plus the new value'
        status = self._enterCmndMode()
        if status != self.XRF_OK:
            return status

        at = at + "0%x" % num
        self._sendCommand(at)
        status = self._waitForOK()
        if status != self.XRF_OK:
            # Missing OK
            self._diagPrintLn("sendATxSetHexNumber, missing OK: %d" % status)
        return status

    def _findCrc(self, line):
        '''
         * Find the CRC in the line of text
         *
         * First step is to find the last comma. And if it is
         * found return the characters after it.
         * To help stripping the CRC later we also return the pointer
         * to the comma before the checksum.
         *
         * \param txt a text string
         * \return true if a CRC was found
        '''
        sepIx = line.rfind(self._fldSep)
        if sepIx < 0:
            return line, None
        return line[:sepIx], line[sepIx + 1:]

    def get_crc16_xmodem(self, message):
        '''
        # Thanks to http://bytes.com/profile/254305/highvelcty/
        # CRC-16-CCITT poly, the CRC scheme used by zmodem protocol
        #
        # Well, it is _not_ the CCITT method, but it is the XMODEM
        # method.
        # And it matches _crc_xmodem_update from libc-avr
        '''
        poly = 0x1021
        # 16bit operation register, initialized to zeros
        reg = 0x0000
        # pad the end of the message with the size of the poly
        message += '\x00\x00'
        # for each bit in the message
        for b in message:
            mask = 0x80
            while mask > 0:
                # left shift by one
                reg <<= 1
                # input the next bit from the message into the right hand side of the op reg
                if ord(b) & mask:
                    reg += 1
                mask >>= 1
                # if a one popped out the left of the reg, xor reg w/poly
                if reg > 0xffff:
                    # eliminate any one that popped out on the left
                    reg &= 0xffff
                    # xor with the poly, this is the remainder
                    reg ^= poly
        return reg

    def get_crc16_ccitt(self, message):
        '''
        This is the algorithm as described in avr-libc util/crc16.h for _crc_ccitt_update()
        uint16_t
        crc_ccitt_update (uint16_t crc, uint8_t data)
        {
            data ^= lo8 (crc);
            data ^= data << 4;

            return ((((uint16_t)data << 8) | hi8 (crc)) ^ (uint8_t)(data >> 4) 
                    ^ ((uint16_t)data << 3));
        }
        '''
        crc = 0xffff
        for b in message:
            # Do the crc_ccitt_update for each byte
            b = ord(b)
            b ^= crc & 0xff
            b ^= b << 4
            b = b & 0xff
            crc = ((b << 8) | (crc >> 8)) ^ (b >> 4) ^ (b << 3)
            crc &= 0xffff
        return crc

    def _millis(self):
        return time.time() * 1000

    def _delay(self, nrMillis):
        time.sleep(nrMillis / 1000.)

    def _timedOut(self, tsMax):
        millis = self._millis()
        return millis - tsMax >= 0

    def _diagPrintLn(self, txt):
        self._diagPrint(txt + '\n')

    def _diagPrint(self, txt):
        sys.stdout.write(txt)
        sys.stdout.flush()

if __name__ == '__main__':
    hub = XRF('/dev/ttyUSB1')
    hub.init('HUB', 0x5AA5)
