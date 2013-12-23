/*
 * xrf_master.ino
 *
 * This is an example sketch to demonstrate the usage of the XRF library.
 */


#define XRF_DEMO_PANID      0x5AA5
#define XRF_DEMO_MASTER         1
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

//################  forward declare  ###############
void readCommand();
int findCmndIx(const char *data);
void srvHello(const char *data);
void srvTs(const char *data);

void setup()
{
  Serial.begin(9600);
  xrf.init(XRF_DEMO_MASTER, Serial);
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

  delay(1000);
  Serial.println("XRF master");
}

void loop()
{
  int nr = xrf.available();
  if (nr > 0) {
    DIAGPRINT(F("data available: ")); DIAGPRINTLN(nr);
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

  status = xrf.receiveData(data, sizeof(data));
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
void srvHello(const char *data)
{
  char line[60];

  DIAGPRINT(F("srvHello: '")); DIAGPRINT(data); DIAGPRINTLN('\'');

  // TODO Store this slave ID in a list

  // TODO Send an acknowledge
  strcpy(line, data);
  strcat(line, ",ack");
  (void)xrf.sendData(line);
}

/*
 * Service the "ts" command
 *
 * The format of the remainder of the line must be:
 *   <slave id>
 * The answer is to repeat that part and to append ",ack"
 */
void srvTs(const char *data)
{
  char line[60];
  char *ptr;

  DIAGPRINT(F("srvTs: '")); DIAGPRINT(data); DIAGPRINTLN('\'');

  ptr = line;
  strcpy(ptr, data);
  ptr += strlen(data);
  *ptr++ = ',';
  uint32_t ts = rtc.now().getEpoch();
  ultoa(ts, ptr, 10);
  (void)xrf.sendData(line);
}
