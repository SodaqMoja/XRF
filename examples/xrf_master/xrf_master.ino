/*
 * xrf_master.ino
 *
 * This is an example sketch to demonstrate the usage of the XRF library.
 */


#define XRF_DEMO_PANID          0x5AA5
#define XRF_REQUEST_PREFIX      "M,"

#include <string.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Sodaq_DS3231.h>
#include <XRF.h>

#include "Diag.h"
#include "Utils.h"

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

struct SlaveInfo_t
{
  char name[10];
  uint16_t uploadOffset;

  // The rest is the data record per slave

  float battVolt;
};
typedef struct SlaveInfo_t SlaveInfo_t;
SlaveInfo_t slaves[4];
size_t nrSlaves;

// A flag to indicate that a WDT interrupt happened.
bool wdtTicked;

bool doneRtcUpdate;

// How long does a wake up last?
// IMPORTANT. This amount determines the interval of a slave trying to reconnect.
const uint16_t masterWakeUpPeriod = 20;
// What is the time between two wake ups?
const uint16_t masterWakeUpInterval = 1 * 60;
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
void setupWatchDog();
void gotoSleep();

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
  /* Clear WDRF in MCUSR */
  MCUSR &= ~_BV(WDRF);

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

  // Setting panID must succeed
  while (xrf.setPanID(XRF_DEMO_PANID) != XRF_OK) {
  }
  xrf.setSleepMode(1, XBEE_DTR);
  (void)xrf.leaveCmndMode();
  Serial.println("XRF master");

  uint32_t ts = rtc.now().getEpoch();
  // Start wake up period right away.
  nextWakeUp = ts;
  // Do upload in 30 seconds from now
  nextUpload = ts + 30;

  setupWatchDog();

  // Enable interrupts
  interrupts();
}

void loop()
{
  if (wdtTicked) {
    wdt_reset();
    WDTCSR |= _BV(WDIE);                // Make sure WDIE is set (see setupWatchdog)

    wdtTicked = false;
  }

  if (!doneRtcUpdate) {
    doRtcUpdate();
  }

  uint32_t ts = rtc.now().getEpoch();
  if (ts >= nextWakeUp || ts >= nextUpload) {
    DIAGPRINTLN("XRF master, start wakeup");
    xrf.wakeUp();
    delay(100);                 // FIXME How much is needed, if any?
    Serial.print("XRF master, start wakeup "); Serial.println(ts);
    xrf.flushInput();
    // Read commands and execute them
    uint32_t endWakeUp = ts + masterWakeUpPeriod;
    while (rtc.now().getEpoch() < endWakeUp) {
      if (xrf.available() > 0) {
        readCommand();
      }
    }
    ts = rtc.now().getEpoch();
    if (ts > nextWakeUp) {
      nextWakeUp += masterWakeUpInterval;
    }
    if (ts > nextUpload) {
      nextUpload += uploadInterval;
    }
    DIAGPRINTLN("XRF master, end wakeup");
    Serial.println("XRF master, end wakeup");
    delay(1000);
  }

  // Go to sleep
  gotoSleep();
}

//################ watchdog timer ################

// The watchdog timer is used to make timed interrupts
// This is a modified version of wdt_enable() from avr/wdt.h
// Only WDIE is set!!
// Note from the doc: "Executing the corresponding interrupt
// vector will clear WDIE and WDIF automatically by hardware
// (the Watchdog goes to System Reset Mode)
#define my_wdt_enable(value)   \
__asm__ __volatile__ (  \
    "in __tmp_reg__,__SREG__" "\n\t"    \
    "cli" "\n\t"    \
    "wdr" "\n\t"    \
    "sts %0,%1" "\n\t"  \
    "out __SREG__,__tmp_reg__" "\n\t"   \
    "sts %0,%2" "\n\t" \
    : /* no outputs */  \
    : "M" (_SFR_MEM_ADDR(_WD_CONTROL_REG)), \
      "r" (_BV(_WD_CHANGE_BIT) | _BV(WDE)), \
      "r" ((uint8_t) (((value & 0x08) ? _WD_PS3_MASK : 0x00) | \
          _BV(WDIE) | (value & 0x07)) ) \
    : "r0"  \
)

/*
 * Setup the watch dog (wdt)
 */
void setupWatchDog()
{
#if 1
  my_wdt_enable(WDTO_1S);
#else
  wdt_enable(WDTO_1S);
  WDTCSR |= _BV(WDIE);
  /*
   * Now the wdt is in "Interrupt and System Reset Mode".
   * The first time-out in the Watchdog Timer will set WDIF. Executing the corresponding interrupt
   * vector will clear WDIE and WDIF automatically by hardware (the Watchdog goes to System Reset
   * Mode). This is useful for keeping the Watchdog Timer security while using the interrupt. To
   * stay in Interrupt and System Reset Mode, WDIE must be set after each interrupt. This should
   * however not be done within the interrupt service routine itself, as this might compromise
   * the safety-function of the Watchdog System Reset mode. If the interrupt is not executed
   * before the next time-out, a System Reset will be applied.
   */
#endif
}

//################ interrupt ################
/*
 * \brief WDT Interrupt handler
 *
 * This handler doesn't do a lot. Just set a flag.
 * Meanwhile the CPU woke up, so that the program continues
 * in the main loop.
 */
ISR(WDT_vect)
{
  wdtTicked = true;
}

//################ sleep mode ################
void gotoSleep()
{
  xrf.sleep();

  ADCSRA &= ~_BV(ADEN);         // ADC disabled

  /*
  * Possible sleep modes are (see sleep.h):
  #define SLEEP_MODE_IDLE         (0)
  #define SLEEP_MODE_ADC          _BV(SM0)
  #define SLEEP_MODE_PWR_DOWN     _BV(SM1)
  #define SLEEP_MODE_PWR_SAVE     (_BV(SM0) | _BV(SM1))
  #define SLEEP_MODE_STANDBY      (_BV(SM1) | _BV(SM2))
  #define SLEEP_MODE_EXT_STANDBY  (_BV(SM0) | _BV(SM1) | _BV(SM2))
  */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();

  ADCSRA |= _BV(ADEN);          // ADC enabled
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
  // The +1 is to give the master extra time to wake up
  // before are going to send their messages.
  uint32_t ts = nextUpload + 1 + slaves[slaveIx].uploadOffset;
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

  // Extract battery value from the data stream
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
