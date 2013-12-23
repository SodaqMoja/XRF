/*
 * xrf_slave.ino
 *
 * This is an example sketch to demonstrate the usage of the XRF library.
 */


#define XRF_DEMO_PANID          0x5AA5
#define XRF_DEMO_MASTER         1
#define XRF_DEMO_SLAVE1         42
#define XRF_DEMO_REQUEST_PREFIX "M,"
#define XRF_DEMO_SLAVE1_NAME    "S42"

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Sodaq_DS3231.h>
#include <XRF.h>

#include "Diag.h"
#include "Utils.h"

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
bool doneHello;
bool doneTs;

//################  forward declare  ###############
void doSendHello();
void doSendTs();

void setup()
{
  Serial.begin(9600);
  xrf.init(XRF_DEMO_SLAVE1, Serial);
#ifdef ENABLE_DIAG
  diagport.begin(9600);
  xrf.setDiag(diagport);
#endif
  delay(100);
  DIAGPRINTLN("XRF slave");

  Wire.begin();
  rtc.begin();

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
      doSendTs();
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
  strcpy(ptr, XRF_DEMO_SLAVE1_NAME);

  (void)xrf.sendData(line);

  // Wait until we get a line starting with slave name
  ptr = line;
  strcpy(ptr, XRF_DEMO_SLAVE1_NAME);
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
  strcpy(ptr, XRF_DEMO_SLAVE1_NAME);

  (void)xrf.sendData(line);

  // Wait until we get a line starting with slave name
  ptr = line;
  strcpy(ptr, XRF_DEMO_SLAVE1_NAME);
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
      doneTs = true;
    }
  } else {
    DIAGPRINT("receiveData ts failed: "); DIAGPRINTLN(status);
  }
}
