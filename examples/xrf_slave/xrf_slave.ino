/*
 * xrf_slave.ino
 *
 * This is an example sketch to demonstrate the usage of the XRF library.
 */


#define XRF_DEMO_PANID          0x5AA5
#define XRF_DEMO_REQUEST_PREFIX "M,"

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Sodaq_DS3231.h>
#include <Sodaq_dataflash.h>
#include <XRF.h>

#include "Diag.h"
#include "Utils.h"

#define DF_MOSI         11
#define DF_MISO         12
#define DF_SPICLOCK     13
#define DF_SLAVESELECT  10

// Only needed if DIAG is enabled
#define DIAGPORT_RX     4
#define DIAGPORT_TX     5

//#########       diag      #############
#ifdef ENABLE_DIAG
#if defined(UBRRH) || defined(UBRR0H)
// There probably is no other Serial port that we can use
// Use a Software Serial instead
#include <SoftwareSerial.h>
SoftwareSerial diagport(DIAGPORT_RX, DIAGPORT_TX);
#else
#define diagport Serial
#endif
#endif


XRF xrf;
char slaveName[10];     // This must hold 'S' plus a 8 hexdigit number and a \0
bool doneHello;
bool doneTs;

//################  forward declare  ###############
void createSlaveName(uint8_t *buffer, size_t size);
void doSendHello();
void doSendTs();

void dumpBuffer(uint8_t * buf, size_t size);

void setup()
{
  Serial.begin(9600);
  xrf.init(Serial);
#ifdef ENABLE_DIAG
  diagport.begin(9600);
  xrf.setDiag(diagport);
#endif
  delay(100);
  DIAGPRINTLN("XRF slave");

  Wire.begin();
  rtc.begin();
  dflash.init(DF_MISO, DF_MOSI, DF_SPICLOCK, DF_SLAVESELECT);

  {
    uint8_t buffer[128];
    dflash.readSecurityReg(buffer, 128);
    /* An example of the second 64 bytes of the security register
0D0414071A2D1F2600011204FFFFE8FF
303032533636313216140AFFFFFFFFFF
3E3E3E3E3E3C3E3E3E3C3E3C3C3E3E3C
FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
     */
    createSlaveName(buffer + 64, 64);
    DIAGPRINT(F("slaveName: '")); DIAGPRINT(slaveName); DIAGPRINTLN('\'');
  }

  xrf.setPanID(XRF_DEMO_PANID);
  (void)xrf.leaveCmndMode();
}

void loop()
{
  if (!doneTs) {
    // Don't flood the XRF channel
    static int counter;
    if (++counter > 10) {
      counter = 0;

      if (!doneHello) {
        doSendHello();
      }
      if (doneHello) {
        doSendTs();
      }
    } else {
      delay(100);
    }
  }
}

/*
 * Send the "hello" command and wait for an "ack"
 */
void doSendHello()
{
  uint8_t status;
  char line[60];
  char data[60];
  char *ptr = line;

  strcpy(ptr, XRF_DEMO_REQUEST_PREFIX);
  ptr += strlen(ptr);
  strcpy(ptr, "hello");
  ptr += strlen(ptr);
  *ptr++ = ',';
  strcpy(ptr, slaveName);

  (void)xrf.sendData(line);

  // Wait until we get a line starting with slave name
  ptr = line;
  strcpy(ptr, slaveName);
  ptr += strlen(ptr);
  *ptr++ = ',';
  *ptr = '\0';
  status = xrf.receiveData(line, data, sizeof(data));
  if (status == XRF_OK) {
    ptr = data + strlen(line);
    if (strncmp(ptr, "ack", 3) == 0) {
      doneHello = true;
      DIAGPRINTLN("receive hello ack: ");
    }
  } else {
    DIAGPRINT("receiveData hello failed: "); DIAGPRINTLN(status);
  }
}

/*
 * Send the "ts" command and wait for the timestamp from the master
 *
 * If the timestamp is received the RTC is updated with it.
 */
void doSendTs()
{
  uint8_t status;
  char line[60];
  char data[60];
  char *ptr = line;
  char *eptr;

  strcpy(ptr, XRF_DEMO_REQUEST_PREFIX);
  ptr += strlen(ptr);
  strcpy(ptr, "ts");
  ptr += strlen(ptr);
  *ptr++ = ',';
  strcpy(ptr, slaveName);

  (void)xrf.sendData(line);

  // Wait until we get a line starting with slave name
  ptr = line;
  strcpy(ptr, slaveName);
  ptr += strlen(ptr);
  *ptr++ = ',';
  *ptr = '\0';
  status = xrf.receiveData(line, data, sizeof(data));
  if (status == XRF_OK) {
    ptr = data + strlen(line);
    // Expecting a number
    DIAGPRINT(F("ts: '")); DIAGPRINT(data); DIAGPRINTLN('\'');
    uint32_t ts = strtoul(ptr, &eptr, 0);
    if (eptr != ptr) {
      rtc.setEpoch(ts);
      doneTs = true;
    }
  } else {
    DIAGPRINT("receiveData ts failed: "); DIAGPRINTLN(status);
  }
}

/*
 * A modified version of utoa which does the same as printf("%04X", val)
 */
void myUtoa(uint16_t val, char *buf)
{
  if (val < 0x1000) {
    *buf++ = '0';
    if (val < 0x100) {
      *buf++ = '0';
      if (val < 0x10) {
        *buf++ = '0';
      }
    }
  }
  utoa(val, buf, 16);
}

void createSlaveName(uint8_t *buffer, size_t size)
{
  //dumpBuffer(buffer, size);
  uint16_t crc1 = crc16_ccitt(buffer, 16);
  uint16_t crc2 = crc16_ccitt(buffer + 16, 16);
  char *ptr = slaveName;
  *ptr++ = 'S';
  myUtoa(crc1, ptr);
  ptr += 4;
  myUtoa(crc2, ptr);
  // String now has a \0 terminator
}

void dumpBuffer(uint8_t * buf, size_t size)
{
  while (size > 0) {
    size_t size1 = size >= 16 ? 16 : size;
    for (size_t j = 0; j < size1; j++) {
      // Silly Arduino Print has very limited formatting capabilities
      DIAGPRINT((*buf >> 4) & 0xF, HEX);        // High nibble
      DIAGPRINT(*buf & 0xF, HEX);               // Low nibble
      buf++;
    }
    DIAGPRINTLN();
    size -= size1;
  }
}
