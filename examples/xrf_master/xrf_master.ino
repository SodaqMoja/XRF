/*
 * xrf_master.ino
 *
 * This is an example sketch to demonstrate the usage of the XRF library.
 */


#define XRF_DEMO_PANID          0x5AA5
#define XRF_REQUEST_PREFIX      "M,"

#include <string.h>
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
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

struct SlaveInfo_t
{
  char name[10];
  uint16_t uploadOffset;
  float battVolt;
};
typedef struct SlaveInfo_t SlaveInfo_t;
SlaveInfo_t slaves[4];
size_t nrSlaves;

bool doneRtcUpdate;

// How long does a wake up last?
const uint16_t wakeUpPeriod = 20;
// What is the time between two wake ups?
const uint16_t wakeUpInterval = 1 * 60;
// The time for the next wake up
uint32_t nextWakeUp;
// The time for the next upload
uint32_t nextUpload;
// What is the time between two uploads?
#if 0
const uint16_t uploadInterval = 60 * 60;
#else
// make the interval shorter during development / debugging.
const uint16_t uploadInterval = 2 * 60;
#endif

//################  forward declare  ###############
void doRtcUpdate();

void readCommand();
int findCmndIx(const char *data);
void srvHello(const char *name);
void srvTs(const char *data);
void srvNext(const char *data);
void srvBatt(const char *data);

int findSlave(const char *name);
bool addSlave(const char *name);

void setup()
{
  Serial.begin(9600);
  xrf.init(Serial);
#ifdef ENABLE_DIAG
  diagport.begin(9600);
  xrf.setDiag(diagport);
#endif
  delay(100);
  DIAGPRINTLN("XRF master");

  Wire.begin();
  rtc.begin();

  xrf.setPanID(XRF_DEMO_PANID);
  (void)xrf.leaveCmndMode();
  Serial.println("XRF master");

  uint32_t ts = rtc.now().getEpoch();
  // Start wake up period right away.
  nextWakeUp = ts;
  // Do upload in 30 seconds from now
  nextUpload = ts + 30;
}

void loop()
{
  if (!doneRtcUpdate) {
    doRtcUpdate();
  }

  uint32_t ts = rtc.now().getEpoch();
  if (ts >= nextWakeUp || ts >= nextUpload) {
    DIAGPRINTLN("XRF master, start wakeup");
    Serial.print("XRF master, start wakeup "); Serial.println(ts);
    xrf.flushInput();
    // Read commands and execute them
    uint32_t endWakeUp = ts + wakeUpPeriod;
    while (rtc.now().getEpoch() < endWakeUp) {
      if (xrf.available() > 0) {
        readCommand();
      }
    }
    ts = rtc.now().getEpoch();
    if (ts > nextWakeUp) {
      nextWakeUp += wakeUpInterval;
    }
    if (ts > nextUpload) {
      nextUpload += uploadInterval;
    }
    DIAGPRINTLN("XRF master, end wakeup");
    Serial.println("XRF master, end wakeup");
  }

  // Go to sleep
}

/*
 * Get the correct time for RTC
 */
void doRtcUpdate()
{
  // TODO

  doneRtcUpdate = true;
  uint32_t ts = rtc.now().getEpoch();
}

struct Command_t
{
  const char *cmd;
  void (*func)(const char *data);
};
typedef struct Command_t Command_t;
Command_t commands[] = {
    {"hello", srvHello},
    {"ts", srvTs},
    {"next", srvNext},
    {"batt", srvBatt},
};

int findCmndIx(const char *data)
{
  for (size_t ix = 0; ix < sizeof(commands) / sizeof(commands[0]); ++ix) {
    int len = strlen(commands[ix].cmd);
    if (strncasecmp(commands[ix].cmd, data, len) == 0) {
      return ix;
    }
  }
  return -1;
}

void readCommand()
{
  uint8_t status;
  char data[60];
  char *ptr;
  int len;

  status = xrf.receiveData(XRF_REQUEST_PREFIX, data, sizeof(data));
  if (status == XRF_OK) {
    DIAGPRINT(F("data received: '")); DIAGPRINT(data); DIAGPRINTLN('\'');
    //Serial.print(F("data received: '")); Serial.print(data); Serial.println('\'');
    // Hmm. We have something. Is it for us?
    len = strlen(XRF_REQUEST_PREFIX);
    if (strncmp(data, XRF_REQUEST_PREFIX, len) == 0) {
      // Yes, it is
      ptr = data + len;
      int cmdIx = findCmndIx(ptr);
      if (cmdIx >= 0) {
        if (commands[cmdIx].func) {
          // Skip the command text and the comma
          ptr += strlen(commands[cmdIx].cmd);
          if (*ptr == ',') {
            ++ptr;
          }
          commands[cmdIx].func(ptr);
        }
      }
    }
  }
}

/*
 * Service the "hello" command
 *
 * The format of the remainder of the line must be:
 *   <slave id>
 * The answer is formatted as follows:
 *   <slave id> ',' "ack"
 */
void srvHello(const char *name)
{
  char line[60];

  DIAGPRINT(F("srvHello: '")); DIAGPRINT(name); DIAGPRINTLN('\'');

  int slaveIx = findSlave(name);
  if (slaveIx < 0) {
    // Store this slave ID in a list
    if (!addSlave(name)) {
      DIAGPRINTLN(F("srvHello slave name not accepted"));
      return;
    }
  }

  strcpy(line, name);
  strcat(line, ",ack");
  (void)xrf.sendData(line);
}

/*
 * Service the "ts" command
 *
 * The format of the remainder of the line must be:
 *   <slave id>
 * The answer is formatted as follows:
 *   <slave id> ',' <ts>
 */
void srvTs(const char *name)
{
  char line[60];
  char *ptr;

  DIAGPRINT(F("srvTs: '")); DIAGPRINT(name); DIAGPRINTLN('\'');

  int slaveIx = findSlave(name);
  if (slaveIx < 0) {
    return;
  }

  ptr = line;
  strcpy(ptr, name);
  ptr += strlen(name);
  *ptr++ = ',';
  uint32_t ts = rtc.now().getEpoch();
  ultoa(ts, ptr, 10);
  (void)xrf.sendData(line);
}

/*
 * Service the "next" command
 *
 * The format of the remainder of the line must be:
 *   <slave id>
 *   <slave id> ',' <ts> ',' <interval>
 */
void srvNext(const char *name)
{
  char line[60];
  char *ptr;

  DIAGPRINT(F("srvNext: '")); DIAGPRINT(name); DIAGPRINTLN('\'');

  int slaveIx = findSlave(name);
  if (slaveIx < 0) {
    return;
  }

  ptr = line;
  strcpy(ptr, name);
  ptr += strlen(name);
  *ptr++ = ',';
  uint32_t ts = nextUpload + slaves[slaveIx].uploadOffset;
  ultoa(ts, ptr, 10);
  ptr += strlen(ptr);
  *ptr++ = ',';
  ultoa(uploadInterval, ptr, 10);
  (void)xrf.sendData(line);
}

/*
 * Service the "batt" command
 *
 * The format of the remainder of the line must be:
 *   <slave id> ',' <value>
 * The answer is formatted as follows:
 *   <slave id> ',' "ack"
 */
void srvBatt(const char *data)
{
  char line[60];
  const char *dptr;
  char *ptr;
  char *eptr;

  DIAGPRINT(F("srvBatt: '")); DIAGPRINT(data); DIAGPRINTLN('\'');

  // Get slave name
  for (ptr = line, dptr = data; *dptr && *dptr != ','; ++dptr) {
    *ptr++ = *dptr;
  }
  *ptr = '\0';

  int slaveIx = findSlave(line);
  if (slaveIx < 0) {
    return;
  }

  // TODO Get battery value from the data
  if (*dptr == ',') {
    ++dptr;
  }
  float battVolt = strtod(dptr, &eptr);
  if (eptr != dptr) {
    slaves[slaveIx].battVolt = battVolt;
    strcat(line, ",ack");
    (void)xrf.sendData(line);
  }
}

int findSlave(const char *name)
{
  for (size_t i = 0; i < nrSlaves; ++i) {
    SlaveInfo_t *slave = &slaves[i];
    if (strcmp(slave->name, name) == 0) {
      return i;
    }
  }
  DIAGPRINT(F("unknown slave: '")); DIAGPRINT(name); DIAGPRINTLN('\'');
  return -1;
}

bool addSlave(const char *name)
{
  if (nrSlaves >= sizeof(slaves) / sizeof(slaves[0])) {
    // Already reached maximum
    return false;
  }

  SlaveInfo_t *next = &slaves[nrSlaves];
  // This strips the name to length of a maximum of 8.
  // Perhaps we must refuse this name.
  memset(next->name, 0, sizeof(next->name));
  strncpy(next->name, name, sizeof(next->name) - 1);
  // Give each slave a different offset to avoid collision
  next->uploadOffset = (nrSlaves + 1) * 1;
  ++nrSlaves;
  return true;
}
