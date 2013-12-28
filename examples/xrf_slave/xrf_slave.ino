/*
 * xrf_slave.ino
 *
 * This is an example sketch to demonstrate the usage of the XRF library.
 */


#define XRF_DEMO_PANID          0x5AA5
#define XRF_REQUEST_PREFIX      "M,"
#define ADC_AREF                3.3     // DEFAULT see wiring_analog.c

#include <string.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

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

// A flag to indicate that a WDT interrupt happened.
bool wdtTicked;

bool doneHello;
bool doneTs;

const uint16_t masterWakeUpPeriod = 20;
// The time for the next wake up
uint32_t nextWakeUp;
// The time for the next upload
uint32_t nextUpload;
// What is the time between two uploads?
uint16_t uploadInterval;

size_t failedCounter;
const size_t maxFailedCounter = 10;
const size_t maxRetryCount = 3;

//################  forward declare  ###############
void setupWatchDog();
void gotoSleep();

void createSlaveName(uint8_t *buffer, size_t size);
void doSendHello();
void doSendTs();
void doAskNextUpload();
void doUpload();
bool sendCommandAndWaitForReply(const char *cmd, char *data, size_t size);
bool sendKeyValueAndWaitForAck(const char *parm, const char *val);
bool sendKeyValueAndWaitForAck(const char *parm, uint32_t val);
bool sendKeyValueAndWaitForAck(const char *parm, float val);

void bumpFailedCounter();
void resetFailedCounter();

void redoHello();
void setRtc(uint32_t ts);

float getRealBatteryVoltage();
void dumpBuffer(uint8_t * buf, size_t size);

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

  // Setting panID must succeed
  while (xrf.setPanID(XRF_DEMO_PANID) != XRF_OK) {
  }
  xrf.setSleepMode(1, XBEE_DTR);
  (void)xrf.leaveCmndMode();
  Serial.print("XRF slave "); Serial.println(slaveName);

  redoHello();

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
  if ((nextWakeUp && ts >= nextWakeUp) || (nextUpload && ts >= nextUpload)) {
    DIAGPRINTLN("XRF slave, start wakeup");
    //DIAGPRINT("nextUpload: "); DIAGPRINTLN(nextUpload);
    //DIAGPRINT("nextWakeUp: "); DIAGPRINTLN(nextWakeUp);
    // We need to connect to the master
    xrf.wakeUp();
    delay(100);
    Serial.print("XRF slave, start wakeup "); Serial.println(rtc.now().getEpoch());

    if (!doneHello) {
      doSendHello();
      if (!doneHello) {
        goto endLoopRetry;
      } else {
        // No need for extra wake up
        nextWakeUp = 0;
      }
    }

    if (!doneTs) {
      doSendTs();
      if (!doneTs) {
        goto endLoopRetry;
      } else {
        // No need for extra wake up
        nextWakeUp = 0;
      }
    }

    if (nextUpload == 0) {
      // We need to ask the master when the next upload is
      doAskNextUpload();
      if (nextUpload == 0) {
        goto endLoopRetry;
      } else {
        // No need for extra wake up
        nextWakeUp = 0;
      }
    }

    // Is it time for the next upload?
    ts = rtc.now().getEpoch();
    if (ts >= nextUpload) {
      doUpload();

      Serial.print("XRF slave ");
      Serial.print(slaveName);
      Serial.println(", done upload");
      delay(1000);
    }
  }
  goto endLoop;

endLoopRetry:
  nextWakeUp = rtc.now().getEpoch() + masterWakeUpPeriod - 2;   // TODO The needs tuning -1? -2? -3?

endLoop:
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
      setRtc(ts);
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
void doAskNextUpload()
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
  nextWakeUp = rtc.now().getEpoch();
}

void setRtc(uint32_t ts)
{
  if (rtc.now().getEpoch() != ts) {
    // Update the RTC
    rtc.setEpoch(ts);
    // Make sure we request new upload time, because RTC has changed
    nextUpload = 0;
  }
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
