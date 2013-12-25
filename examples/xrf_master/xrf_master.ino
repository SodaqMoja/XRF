/*
 * xrf_master.ino
 *
 * This is an example sketch to demonstrate the usage of the XRF library.
 */


#define XRF_DEMO_PANID          0x5AA5
#define XRF_DEMO_REQUEST_PREFIX "M,"

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
};
typedef struct SlaveInfo_t SlaveInfo_t;
SlaveInfo_t slaves[4];
size_t nrSlaves;

uint32_t nextUpload = 1388011383;

//################  forward declare  ###############
void readCommand();
int findCmndIx(const char *data);
void srvHello(const char *name);
void srvTs(const char *data);
void srvNext(const char *data);

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

  // While debugging, send the message to the world.
  delay(1000);
  Serial.println("XRF master");
}

void loop()
{
  int nr = xrf.available();
  if (nr > 0) {
    //DIAGPRINT(F("data available: ")); DIAGPRINTLN(nr);
    readCommand();
  }
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

  status = xrf.receiveData(XRF_DEMO_REQUEST_PREFIX, data, sizeof(data));
  if (status == XRF_OK) {
    DIAGPRINT(F("data received: '")); DIAGPRINT(data); DIAGPRINTLN('\'');
    //Serial.print(F("data received: '")); Serial.print(data); Serial.println('\'');
    // Hmm. We have something. Is it for us?
    len = strlen(XRF_DEMO_REQUEST_PREFIX);
    if (strncmp(data, XRF_DEMO_REQUEST_PREFIX, len) == 0) {
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
 * The answer is to repeat that part and to append ",ack"
 */
void srvHello(const char *name)
{
  char line[60];

  DIAGPRINT(F("srvHello: '")); DIAGPRINT(name); DIAGPRINTLN('\'');

  // Store this slave ID in a list
  if (!addSlave(name)) {
    DIAGPRINTLN(F("srvHello slave name not accepted"));
    return;
  }

  // TODO Send an acknowledge
  strcpy(line, name);
  strcat(line, ",ack");
  (void)xrf.sendData(line);
}

/*
 * Service the "ts" command
 *
 * The format of the remainder of the line must be:
 *   <slave id>
 * The answer is to repeat that part and to append the current timestamp
 */
void srvTs(const char *name)
{
  char line[60];
  char *ptr;

  DIAGPRINT(F("srvTs: '")); DIAGPRINT(name); DIAGPRINTLN('\'');

  int slaveIx = findSlave(name);
  if (slaveIx < 0) {
    DIAGPRINTLN(F("srvTs unknown slave"));
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
 * The answer is to repeat that part and to append the timestamp for the next upload
 */
void srvNext(const char *name)
{
  char line[60];
  char *ptr;

  DIAGPRINT(F("srvNext: '")); DIAGPRINT(name); DIAGPRINTLN('\'');

  int slaveIx = findSlave(name);
  if (slaveIx < 0) {
    DIAGPRINTLN(F("srvNext unknown slave"));
    return;
  }

  ptr = line;
  strcpy(ptr, name);
  ptr += strlen(name);
  *ptr++ = ',';
  uint32_t ts = nextUpload + slaves[slaveIx].uploadOffset;
  ultoa(ts, ptr, 10);
  (void)xrf.sendData(line);
}

int findSlave(const char *name)
{
  for (size_t i = 0; i < nrSlaves; ++i) {
    SlaveInfo_t *slave = &slaves[i];
    if (strcmp(slave->name, name) == 0) {
      return i;
    }
  }
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
  ++nrSlaves;
  return true;
}
