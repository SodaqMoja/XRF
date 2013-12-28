/*
 * xrf_slave.ino
 *
 * This is an example sketch to demonstrate the usage of the XRF library.
 */


#define XRF_DEMO_PANID          0x5AA5
#define XRF_REQUEST_PREFIX      "M,"
#define ADC_AREF                3.3     // DEFAULT see wiring_analog.c

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

#define BATVOLTPIN      A7
#define BATVOLT_R1      10              // in fact 10M
#define BATVOLT_R2      2               // in fact 2M

// The Xbee DTR is connected to the XRF sleep pin
#define XBEE_DTR        7

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
uint32_t nextUpload;
uint16_t uploadInterval;
size_t failedCounter;
const size_t maxFailedCounter = 10;
const size_t maxRetryCount = 3;

//################  forward declare  ###############
void createSlaveName(uint8_t *buffer, size_t size);
void doSendHello();
void doSendTs();
void doAskUpload();
void doUpload();
bool sendCommandAndWaitForReply(const char *cmd, char *data, size_t size);
bool sendKeyValueAndWaitForAck(const char *parm, const char *val);
bool sendKeyValueAndWaitForAck(const char *parm, uint32_t val);
bool sendKeyValueAndWaitForAck(const char *parm, float val);

void redoHello();
void bumpFailedCounter();
void resetFailedCounter();

float getRealBatteryVoltage();
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
  Serial.print("XRF slave ");
  Serial.println(slaveName);

  xrf.setSleepMode(1, XBEE_DTR);
  xrf.setPanID(XRF_DEMO_PANID);
  (void)xrf.leaveCmndMode();
}

void loop()
{
  if (!doneHello) {
    doSendHello();
    if (!doneHello) {
      delay(1000);
    }
    return;
  }

  if (!doneTs) {
    doSendTs();
    if (!doneTs) {
      delay(1000);
    }
    return;
  }

  if (nextUpload == 0) {
    // We need to ask the master when the next upload is
    doAskUpload();
    return;
  }

  // Is it time for the next upload?
  uint32_t ts = rtc.now().getEpoch();
  if (ts >= nextUpload) {
    Serial.print("XRF slave ");
    Serial.print(slaveName);
    Serial.print(", start wakeup ");
    Serial.println(ts);

    // TODO Do next upload
    doUpload();

    Serial.print("XRF slave ");
    Serial.print(slaveName);
    Serial.println(", done upload");
  }

  // Go to sleep
}

/*
 * Send the "hello" command and wait for an "ack"
 */
void doSendHello()
{
  char data[60];
  char *ptr;

  if (sendCommandAndWaitForReply("hello", data, sizeof(data))) {
    ptr = data;
    if (strncmp(ptr, "ack", 3) == 0) {
      doneHello = true;
      DIAGPRINTLN("receive hello ack: ");
    } else {
    }
  }
}

/*
 * Send the "ts" command and wait for the timestamp from the master
 *
 * If the timestamp is received, the RTC is updated with it.
 */
void doSendTs()
{
  char data[60];
  char *ptr;
  char *eptr;

  if (sendCommandAndWaitForReply("ts", data, sizeof(data))) {
    // Expecting a number
    ptr = data;
    // Expecting a number
    uint32_t ts = strtoul(ptr, &eptr, 0);
    if (eptr != ptr) {
      if (rtc.now().getEpoch() != ts) {
        // Update the RTC
        rtc.setEpoch(ts);
        // Make sure we request new upload time, because RTC has changed
        nextUpload = 0;
      }
      doneTs = true;
    } else {
      //DIAGPRINTLN("Invalid number");
    }
  }
}

/*
 * Send the "next" command
 *
 * The answer from the master should have the timestamp for the
 * next upload, and also the upload interval (in seconds)
 */
void doAskUpload()
{
  char data[60];
  char *ptr;
  char *eptr;

  if (sendCommandAndWaitForReply("next", data, sizeof(data))) {
    // Expecting a number
    ptr = data;
    uint32_t ts = strtoul(ptr, &eptr, 0);
    if (eptr != ptr) {
      nextUpload = ts;
      DIAGPRINT("nextUpload: "); DIAGPRINTLN(nextUpload);
      if (*eptr == ',') {
        ++eptr;
        ptr = eptr;
        ts = strtoul(ptr, &eptr, 0);
        if (eptr != ptr) {
          uploadInterval = ts;
          DIAGPRINT("uploadInterval: "); DIAGPRINTLN(uploadInterval);
        }
      }
    } else {
      // Invalid number
      //DIAGPRINTLN("Invalid number");
    }
  }
}

/*
 * Upload data to the master
 *
 * The collected data will be sent to the master, and if it was
 * done successful the time for the next upload is incremented
 * with the interval
 */
void doUpload()
{
  // TODO
  DIAGPRINT("doUpload: "); DIAGPRINTLN(nextUpload);

  bool ok = false;

  float batVolt = getRealBatteryVoltage();
  for (size_t i = 0; i < maxRetryCount; ++i) {
    if (sendKeyValueAndWaitForAck("batt", batVolt)) {
      ok = true;
      break;
    }
  }
  if (!ok) {
    redoHello();
    return;
  }

  if (uploadInterval) {
    nextUpload += uploadInterval;
  } else {
    // This will trigger to ask for a new next upload
    nextUpload = 0;
  }
}

/*
 * Send a simple command and wait for the reply
 */
bool sendCommandAndWaitForReply(const char *cmd, char *data, size_t size)
{
  bool retval = false;
  uint8_t status;
  char line[60];
  char *ptr = line;

  // A bit clumsy code, because using snprintf causes much larger code size
  strcpy(ptr, XRF_REQUEST_PREFIX);
  ptr += strlen(ptr);
  strcpy(ptr, cmd);
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
  status = xrf.receiveData(line, data, size);
  if (status == XRF_OK) {
    resetFailedCounter();
    // Strip our slave prefix.
    ptr = data + strlen(line);
    DIAGPRINT(F("reply: '")); DIAGPRINT(ptr); DIAGPRINTLN('\'');
    // Move the bytes to start of buffer. This should be safe.
    strcpy(data, ptr);
    retval = true;
  } else {
    DIAGPRINT("receiveData failed: "); DIAGPRINTLN(status);
    bumpFailedCounter();
  }
  return retval;
}

/*
 * Send a key and value and wait for the reply
 */
bool sendKeyValueAndWaitForAck(const char *parm, const char *val)
{
  bool retval = false;
  uint8_t status;
  char line[60];
  char data[40];
  char *ptr = line;

  // A bit clumsy code, because using snprintf causes much larger code size
  strcpy(ptr, XRF_REQUEST_PREFIX);
  ptr += strlen(ptr);
  strcpy(ptr, parm);
  ptr += strlen(ptr);
  *ptr++ = ',';
  strcpy(ptr, slaveName);
  ptr += strlen(ptr);
  *ptr++ = ',';
  strcpy(ptr, val);
  (void)xrf.sendData(line);

  // Wait until we get a line starting with slave name
  ptr = line;
  strcpy(ptr, slaveName);
  ptr += strlen(ptr);
  *ptr++ = ',';
  *ptr = '\0';
  status = xrf.receiveData(line, data, sizeof(data));
  if (status == XRF_OK) {
    resetFailedCounter();
    // Strip our slave prefix.
    ptr = data + strlen(line);
    DIAGPRINT(F("reply: '")); DIAGPRINT(ptr); DIAGPRINTLN('\'');
    // We're just expecting "ack"
    if (strcmp(ptr, "ack") == 0) {
      retval = true;
    }
  } else {
    DIAGPRINT("receiveData failed: "); DIAGPRINTLN(status);
    bumpFailedCounter();
  }
  return retval;
}

/*
 * Send a key and value and wait for the reply
 */
bool sendKeyValueAndWaitForAck(const char *parm, uint32_t val)
{
  char buffer[16];
  ultoa(val, buffer, 10);
  return sendKeyValueAndWaitForAck(parm, buffer);
}

/*
 * Send a key and value and wait for the reply
 */
bool sendKeyValueAndWaitForAck(const char *parm, float val)
{
  char buffer[16];
  dtostrf(val, -1, 2, buffer);
  return sendKeyValueAndWaitForAck(parm, buffer);
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

void bumpFailedCounter()
{
  if (++failedCounter > maxFailedCounter) {
    redoHello();
  }
}

void resetFailedCounter()
{
  failedCounter = 0;
}

void redoHello()
{
  doneHello = false;
  doneTs = false;
  nextUpload = 0;
}

/*
 * \brief Read the battery voltage and compute actual voltage
 */
float getRealBatteryVoltage()
{
  /*
   * This pin is connected to the middle of a 10M and 2M resistor
   * that are between Vcc and GND.
   * So actual battery voltage is:
   *    <adc value> * 1023. / AREF * (10+2) / 2
   */
  uint16_t batteryVoltage = analogRead(BATVOLTPIN);
  return (ADC_AREF / 1023.) * batteryVoltage * ((BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2);
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
