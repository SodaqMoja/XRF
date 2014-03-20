/*
 * xrf_master.ino
 *
 * This is an example sketch to demonstrate the usage of the XRF library.
 */

#include <avr/sleep.h>
#include <avr/wdt.h>

#include <Arduino.h>
#include <Wire.h>
#include <Sodaq_DS3231.h>
#include <Sodaq_dataflash.h>
#include <SoftwareSerial.h>
#include <XRF.h>

#include "Diag.h"
#include "Utils.h"

#define ADC_AREF                3.3     // DEFAULT see wiring_analog.c

// Default panID is 0x5AA5. That makes it simple to plug in a monitor
// For more security via obscurity we could pick another
#define XRF_PANID               0x5AA5 // 0x1405
#define XRF_MASTER_NAME         "M"
#define MAX_NR_SLAVES           10

#define DF_MOSI         11
#define DF_MISO         12
#define DF_SPICLOCK     13
#define DF_SLAVESELECT  10

#define BATVOLTPIN      A7
#define BATVOLT_R1      10              // in fact 10M
#define BATVOLT_R2      2               // in fact 2M

// The Xbee DTR is connected to the XRF sleep pin
#define XBEEDTR_PIN     7

#define GROVEPWR_PIN    6
#define GROVEPWR_OFF    LOW
#define GROVEPWR_ON     HIGH

// Only needed if DIAG is enabled
#define DIAGPORT_RX     4
#define DIAGPORT_TX     5


//#########       variables      #############
/*
 * This is the data record as we communicate between
 * gms_slave and gms_master.
 */
struct DeviceDataRecord_t
{
  uint32_t ts;
  float battVolt;
  float temp;
};
typedef struct DeviceDataRecord_t DeviceDataRecord_t;

#define DEV_NAME_LEN 10
struct DataRecord_t
{
  char devName[DEV_NAME_LEN];
  DeviceDataRecord_t	dr;
};
typedef struct DataRecord_t DataRecord_t;
struct SlaveInfo_t
{
  //char name[10];
  uint16_t uploadOffset;
};
typedef struct SlaveInfo_t SlaveInfo_t;
SlaveInfo_t slaves[MAX_NR_SLAVES + 1];
DataRecord_t dataRecords[MAX_NR_SLAVES + 1];    // One extra for the master
size_t nrSlaves;

// Use a generic name "M" so that it can be preconfigured in the slaves
XRF xrf(Serial, XRF_PANID, XRF_MASTER_NAME);
// TODO make this point to dataRecords[0].devName to save space
char masterName[DEV_NAME_LEN];    // This must hold 'M' plus a 8 hexdigit number and a \0


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
uint32_t nextSlaveUpload;
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

void createDeviceName(char *name, char prefix);
void doTimedActions();
void initTimedActions(bool isReset=false);
void doGetServerTS();
void doReadSensors();
void consumeTimestamp(const char *data);
void clearDataRecords();

float getRealBatteryVoltage();

void doRtcUpdate();
void setRtc(uint32_t newTs);
void rtcChangedActions(uint32_t ts);

void readCommand();
int findCmndIx(const char *data);
void srvHello(const char *source, const char *data);
void srvTs(const char *source, const char *data);
void srvNext(const char *source, const char *data);
void srvData(const char *source, const char *data);

int findSlave(const char *name);
bool addSlave(const char *name);

void myUtoa(uint16_t val, char *buf);
void myUtoa(uint8_t val, char *buf);
void bin2hex(char *ptr, size_t dstSize, uint8_t *data, size_t srcSize);
uint16_t hex2Bin(const char *str, int width);

//#########       diag      #############
#ifdef ENABLE_DIAG
#include <SoftwareSerial.h>
SoftwareSerial diagport(DIAGPORT_RX, DIAGPORT_TX);
#endif

void setup()
{
  /* Clear WDRF in MCUSR */
  MCUSR &= ~_BV(WDRF);

  pinMode(GROVEPWR_PIN, OUTPUT);
  digitalWrite(GROVEPWR_PIN, GROVEPWR_OFF);

#ifdef ENABLE_DIAG
  diagport.begin(9600);
#endif
  DIAGPRINTLN("XRF master");

  Wire.begin();
  rtc.begin();
  dflash.init(DF_MISO, DF_MOSI, DF_SPICLOCK, DF_SLAVESELECT);
  createDeviceName(masterName, 'M');
  DIAGPRINT(F("masterName: '")); DIAGPRINT(masterName); DIAGPRINTLN('\'');
  strcpy(dataRecords[0].devName, masterName);
  nrSlaves = 1;

  Serial.begin(9600);
#ifdef ENABLE_DIAG
  xrf.setDiag(diagport);
#endif
  // Sleep mode is important. We must set it before anything else.
  // We want this to succeed
  while (xrf.setSleepMode(2, XBEEDTR_PIN) != XRF_OK) {
    delay(1000);
  }
  xrf.init();

  Serial.print("XRF master "); Serial.println(masterName);

  rtcChangedActions(rtc.now().getEpoch());

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

  uint32_t ts = rtc.now().getEpoch();
  if (ts >= nextWakeUp || ts >= nextSlaveUpload) {
    bool expectedSlaveUpload = ts >= nextSlaveUpload;
    if (!doneRtcUpdate) {
      // Get TS from master2. Retry a few times if it failed.
      for (int i = 0; !doneRtcUpdate && i < 5; ++i) {
        doRtcUpdate();
      }
      if (!doneRtcUpdate) {
        // This must be throttled if it didn't succeed.
        // That is, we need to sleep.
        ts = rtc.now().getEpoch();
        nextWakeUp = ts + 10;
        goto endSleep;
      }
    }

    DIAGPRINTLN("XRF master, start wakeup");
    xrf.wakeUp();
    delay(100);                 // FIXME How much is needed, if any?
    Serial.print("XRF master, start wakeup "); Serial.println(ts);
    Serial.print("XRF master, nextSlaveUpload "); Serial.println(nextSlaveUpload);
    xrf.flushInput();

    if (expectedSlaveUpload) {
      // Update the next upload so that we can sent it
      // in case it is requested.
      nextSlaveUpload += uploadInterval;
    }

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

    if (expectedSlaveUpload) {
      // Get sensor data on this device too
      doReadSensors();

      // Clear slave data records, and master data record (index 0)
      clearDataRecords();
    }
    DIAGPRINTLN("XRF master, end wakeup");
    Serial.println("XRF master, end wakeup");
  }

endSleep:
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

//############# timed actions ################
struct TimedAction_t
{
  const char    *name;
  void          (*func)();
  uint32_t      interval;
  bool          atStart;
  uint32_t      triggerTS;
};
typedef struct TimedAction_t TimedAction_t;
TimedAction_t   timedActions[] = {
    { "GetServerTS", doGetServerTS,     1 * 60 * 60, true },
    //{ "GetSensorData", doReadSensors,   5 * 60 },
};

// Initialize all timed actions
void initTimedActions(bool isReset)
{
  uint32_t ts;
  for (size_t ix = 0; ix < sizeof(timedActions) / sizeof(timedActions[0]); ++ix) {
    TimedAction_t * ptr = &timedActions[ix];
    if (ptr->func) {
      ts = rtc.now().getEpoch();
      if (ptr->atStart) {
        // Do the first action as soon as possible
        ptr->triggerTS = ts;
      } else {
        ptr->triggerTS = ts + ptr->interval;
      }
      DIAGPRINT(ptr->name); DIAGPRINT(":"); DIAGPRINTLN(ptr->triggerTS);
    }
  }
}

void doTimedActions()
{
  uint32_t ts = rtc.now().getEpoch();
  //DIAGPRINT(F("doTimedActions:")); DIAGPRINTLN(ts);
  for (size_t ix = 0; ix < sizeof(timedActions) / sizeof(timedActions[0]); ++ix) {
    TimedAction_t * ptr = &timedActions[ix];
    if (ptr->triggerTS && (long)(ts - ptr->triggerTS) >= 0) {
      // Time to take action
      if (ptr->func) {
        // If the time for the next interval has already passed then
        // reset using the current time
        if ((long)(ptr->triggerTS - ts) < 0) {
          ts = rtc.now().getEpoch();
          ptr->triggerTS = ts + ptr->interval;
        } else {
          ptr->triggerTS += ptr->interval;
        }
        ptr->func();
      }
    }
  }
}

void doGetServerTS()
{
  // This will trigger the request for an update
  uint32_t ts = rtc.now().getEpoch();
  Serial.print(masterName); Serial.print(' '); Serial.print(ts); Serial.println("doGetServerTS");
  doneRtcUpdate = false;
}

void consumeTimestamp(const char *data)
{
  char *eptr;
  if (*data == ',') {
    ++data;
  }
  uint32_t ts = strtoul(data, &eptr, 0);
  if (eptr != data) {
    setRtc(ts);
    doneRtcUpdate = true;
  }
}

/*
 * Read all sensor on this device (the master uses index 0 of dataRecords[])
 */
void doReadSensors()
{
  DeviceDataRecord_t *dr = &dataRecords[0].dr;
  dr->ts = rtc.now().getEpoch();
  dr->battVolt = getRealBatteryVoltage();
  DIAGPRINT(dr->ts);
  DIAGPRINT(" Battery Voltage: "); DIAGPRINTLN(dr->battVolt);
  Serial.print(masterName); Serial.print(' '); Serial.print(dr->ts);
  Serial.print(" Battery Voltage: "); Serial.println(dr->battVolt);
  dr->temp = rtc.getTemperature();
  DIAGPRINT("RTC Temperature: "); DIAGPRINTLN(dr->temp);
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

/*
 * Get the correct time for RTC
 */
void doRtcUpdate()
{
  // TODO Figure out how to set RTC from a reliable source.
  DIAGPRINTLN(F("doRtcUpdate"));
  doneRtcUpdate = true;
}

void setRtc(uint32_t newTs)
{
  uint32_t oldTs = rtc.now().getEpoch();
  int32_t diffTs = abs(newTs - oldTs);
  // Only update the RTC if it differs more than N seconds
  if (diffTs >= 2) {
    // Update the RTC
    rtc.setEpoch(newTs);
    rtcChangedActions(newTs);
  }
}

/*
 * Update all things that depend on RTC
 */
void rtcChangedActions(uint32_t ts)
{
  // Start wake up period right away.
  nextWakeUp = ts;
  // Do upload in 30 seconds from now
  nextSlaveUpload = ts + 30;
}

struct Command_t
{
  const char *cmd;
  void (*func)(const char *source, const char *data);
};
typedef struct Command_t Command_t;
Command_t commands[] = {
    {"hello", srvHello},
    {"ts", srvTs},
    {"next", srvNext},
    {"data", srvData},
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

/*
 * Read one command from the XRF network
 *
 * The packet must be addressed to us, master, using the "M," prefix
 * The receiveMyData takes care of accepting only packets for us. The
 * received data has the following syntax:
 *   <source> ',' <data>
 * And <data> is the data for us.
 */
void readCommand()
{
  uint8_t status;
  char data[90];
  char source[12];

  status = xrf.receiveData(source, sizeof(source), data, sizeof(data));
  if (status == XRF_OK) {
    DIAGPRINT(F("data received: source='")); DIAGPRINT(source); DIAGPRINT('\'');
    DIAGPRINT(F(" data='")); DIAGPRINT(data); DIAGPRINTLN('\'');
    //Serial.print(F("data received: '")); Serial.print(data); Serial.println('\'');
    int cmdIx = findCmndIx(data);
    if (cmdIx >= 0) {
      if (commands[cmdIx].func) {
        // Strip the command
        char *ptr = data;
        ptr += strlen(commands[cmdIx].cmd);
        if (*ptr == ',') {
          ++ptr;
        }
        strcpy(data, ptr);
        commands[cmdIx].func(source, data);
      }
    }
  }
}

/*
 * Service the "hello" command
 *
 * This is a simple request to let a slave identify itself
 */
void srvHello(const char *source, const char *data)
{
  DIAGPRINT(F("srvHello: source='")); DIAGPRINT(source); DIAGPRINT('\'');
  DIAGPRINT(F(" data='")); DIAGPRINT(data); DIAGPRINTLN('\'');

  int slaveIx = findSlave(source);
  if (slaveIx < 0) {
    // Store this slave ID in a list
    if (!addSlave(source)) {
      DIAGPRINTLN(F("srvHello slave name not accepted"));
      return;
    }
  } else {
    // The slave is already known
  }
}

/*
 * Service the "ts" command
 *
 * The answer is formatted as follows:
 *   <ts>
 */
void srvTs(const char *source, const char *data)
{
  DIAGPRINT(F("srvTs: source='")); DIAGPRINT(source); DIAGPRINT('\'');
  DIAGPRINT(F(" data='")); DIAGPRINT(data); DIAGPRINTLN('\'');

  int slaveIx = findSlave(source);
  if (slaveIx < 0) {
    // Store this slave ID in a list
    if (!addSlave(source)) {
      DIAGPRINTLN(F("srvTs slave name not accepted"));
      return;
    }
  }

  // Send our time stamp
  String line;
  line += rtc.now().getEpoch();
  (void)xrf.sendData(source, line.c_str());
}

/*
 * Service the "next" command
 *
 * The answer is formatted as follows:
 *   <next ts> ',' <interval>
 */
void srvNext(const char *source, const char *data)
{
  DIAGPRINT(F("srvNext: source='")); DIAGPRINT(source); DIAGPRINT('\'');
  DIAGPRINT(F(" data='")); DIAGPRINT(data); DIAGPRINTLN('\'');

  int slaveIx = findSlave(source);
  if (slaveIx < 0) {
    // Store this slave ID in a list
    if (!addSlave(source)) {
      DIAGPRINTLN(F("srvNext slave name not accepted"));
      return;
    }
  }

  uint32_t ts = nextSlaveUpload + 1 + slaves[slaveIx].uploadOffset;
  String line;
  line += ts;
  line += ',';
  line += uploadInterval;
  (void)xrf.sendData(source, line.c_str());
}

/*
 * Service the "data" command
 *
 * The format of the remainder of the line must be:
 *   <slave id> ',' <value>
 * The <value> is a hex representation of the data record,
 * 2 hex digits per byte.
 */
void srvData(const char *source, const char *data)
{
  DIAGPRINT(F("srvData: source='")); DIAGPRINT(source); DIAGPRINT('\'');
  DIAGPRINT(F(" data='")); DIAGPRINT(data); DIAGPRINTLN('\'');

  int slaveIx = findSlave(source);
  if (slaveIx < 0) {
    // Store this slave ID in a list
    if (!addSlave(source)) {
      DIAGPRINTLN(F("srvData slave name not accepted"));
      return;
    }
  }
  // The data is a string of hex digits. The length must
  // be exactly twice the size of the data record structure.
  const char *dptr;
  dptr = data;
  size_t len = strlen(dptr);
  DataRecord_t *dr = &dataRecords[slaveIx];
  uint8_t *bptr = (uint8_t *)dr;
  if ((len / 2) == sizeof(*dr)) {
    for (size_t i = 0; i < len; i += 2) {
      *bptr++ = hex2Bin(dptr, 2);
      dptr += 2;
    }
    DIAGPRINT(F("srvData: devName: '")); DIAGPRINT(dr->devName); DIAGPRINTLN('\'');
    DIAGPRINT(F("srvData: ts ")); DIAGPRINTLN(dr->dr.ts);
    DIAGPRINT(F("srvData: battVolt ")); DIAGPRINTLN(dr->dr.battVolt);
    DIAGPRINT(F("srvData: temp ")); DIAGPRINTLN(dr->dr.temp);
  } else {
    // What to do with this?
    // How can we inform the slave? Should we?
    // TODO We need a messaging system so the slave can ask for it.
    DIAGPRINT(F("srvData: data length mismatch, expected ")); DIAGPRINTLN(sizeof(*dr) * 2);
    DIAGPRINT(F("   received ")); DIAGPRINTLN(len);
  }
}

void createDeviceName(char *name, char prefix)
{
  uint8_t buffer[128];
  dflash.readSecurityReg(buffer, 128);
  /* An example of the second 64 bytes of the security register
0D0414071A2D1F2600011204FFFFE8FF
303032533636313216140AFFFFFFFFFF
3E3E3E3E3E3C3E3E3E3C3E3C3C3E3E3C
FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
   */
  //dumpBuffer(buffer + 64, 64);
  uint16_t crc1 = crc16_ccitt(buffer + 64, 16);
  uint16_t crc2 = crc16_ccitt(buffer + 64 + 16, 16);
  char *ptr = name;
  *ptr++ = prefix;
  myUtoa(crc1, ptr);
  ptr += 4;
  myUtoa(crc2, ptr);
  // String now has a \0 terminator
}

//####################################################################
int findSlave(const char *name)
{
  // Skip the first, which is for the master
  for (size_t i = 0; i < nrSlaves && i < sizeof(dataRecords) / sizeof(dataRecords[0]); ++i) {
    DataRecord_t *dr = &dataRecords[i];
    if (dr->devName[0] == '\0') {
      break;
    }
    if (strcmp(dr->devName, name) == 0) {
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

  size_t slaveIx = nrSlaves;
  SlaveInfo_t *next = &slaves[slaveIx];
  DataRecord_t *nextDr = &dataRecords[slaveIx];
  // This strips the name to length of a maximum of 8.
  // Perhaps we must refuse this name.
  memset(nextDr->devName, 0, sizeof(nextDr->devName));
  strncpy(nextDr->devName, name, sizeof(nextDr->devName) - 1);
  DIAGPRINT(F("Add slave: '")); DIAGPRINT(nextDr->devName); DIAGPRINTLN('\'');
  DIAGPRINT(F("    slaveIx: ")); DIAGPRINTLN(slaveIx);
  // Give each slave a different offset to avoid collision
  // Notice that the first slave has index 1.
  next->uploadOffset = slaveIx * 1;
  ++nrSlaves;
  return true;
}

void clearDataRecords()
{
  for (size_t i = 0; i < nrSlaves && i < sizeof(dataRecords) / sizeof(dataRecords[0]); ++i) {
    DeviceDataRecord_t *dr = &dataRecords[i].dr;
    memset(dr, 0, sizeof(*dr));
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

void myUtoa(uint8_t val, char *buf)
{
  if (val < 0x10) {
    *buf++ = '0';
  }
  utoa(val, buf, 16);
}

/*
 * Create a hex digit representation of a byte buffer
 */
void bin2hex(char *ptr, size_t dstSize, uint8_t *data, size_t srcSize)
{
  if (dstSize > 0) {
    // Leave room for a \0 character
    --dstSize;
  }
  for (size_t i = 0; dstSize >= 2 && i < srcSize; ++i, ++data) {
    myUtoa(*data, ptr);
    ptr += 2;
    dstSize -= 2;
  }
}

uint16_t hex2Bin(const char *str, int width)
{
  uint16_t value = 0;
  for (int i = 0; i < width && *str; ++i, ++str) {
    uint8_t b = 0;
    if (*str >= '0' && *str <= '9') {
      b = *str - '0';
    } else if (*str >= 'A' && *str <= 'F') {
      b = *str - 'A' + 10;
    } else if (*str >= 'a' && *str <= 'f') {
      b = *str - 'a' + 10;
    }
    value <<= 4;
    value |= b;
  }
  return value;
}
