/*
 * xrf_slave.ino
 *
 * This is an example sketch to demonstrate the usage of the XRF library.
 */


#include <string.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include <Arduino.h>
#include <Wire.h>
#include <Sodaq_DS3231.h>
#include <Sodaq_dataflash.h>
#include <SoftwareSerial.h>
#include <XRF.h>

// Our own libraries
#include "Diag.h"
#include "Utils.h"
#include "pindefs.h"

#define ADC_AREF                3.3     // DEFAULT see wiring_analog.c

// Default panID is 0x5AA5. That makes it simple to plug in a monitor
// For more security via obscurity we could pick another
#define XRF_PANID               0x5AA5 // 0x1405
#define XRF_MASTER_NAME         "M"


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

XRF xrf(Serial, XRF_PANID, 10, 500);
char slaveName[DEV_NAME_LEN];     // This must hold 'S' plus a 8 hexdigit number and a \0

DataRecord_t dataRecord;

// A flag to indicate that a WDT interrupt happened.
bool wdtTicked;

bool doneHello;
bool doneTs;

const uint16_t masterWakeUpPeriod = 20;
// The time for the next wake up
uint32_t nextWakeUp;
// The time for the next upload
uint32_t nextSlaveUpload;
// What is the time between two uploads?
uint16_t uploadInterval;

size_t failedCounter;
const size_t maxFailedCounter = 10;
const size_t maxRetryCount = 3;

//################  forward declare  ###############
void setupWatchDog();
void gotoSleep();

void doReadSensors();

void createDeviceName(char *name, char prefix);
void doSendHello();
void doSendTs();
void doAskNextUpload();
void doUpload();
bool sendKeyValueAndWaitForAck(const char *parm, const char *val);
bool sendKeyValueAndWaitForAck(const char *parm, uint32_t val);
bool sendKeyValueAndWaitForAck(const char *parm, float val);
void consumeTimestamp(const char *data);

void redoHello();
void setRtc(uint32_t newTs);

float getRealBatteryVoltage();
void dumpBuffer(uint8_t * buf, size_t size);

void bin2hex(char *ptr, size_t dstSize, uint8_t *data, size_t srcSize);
void myUtoa(uint16_t val, char *buf);
void myUtoa(uint8_t val, char *buf);

void setup()
{
  /* Clear WDRF in MCUSR */
  MCUSR &= ~_BV(WDRF);

  pinMode(GROVEPWR_PIN, OUTPUT);
  digitalWrite(GROVEPWR_PIN, GROVEPWR_OFF);

#ifdef ENABLE_DIAG
  diagport.begin(9600);
#endif
  DIAGPRINTLN("XRF slave");

  Wire.begin();
  rtc.begin();
  dflash.init(DF_MISO, DF_MOSI, DF_SPICLOCK, DF_SLAVESELECT);
  createDeviceName(slaveName, 'S');
  DIAGPRINT(F("slaveName: '")); DIAGPRINT(slaveName); DIAGPRINTLN('\'');
  strcpy(dataRecord.devName, slaveName);

  Serial.begin(9600);
#ifdef ENABLE_DIAG
  xrf.setDiag(diagport);
#endif
  // Sleep mode is important. We must set it before anything else.
  // We want this to succeed
  while (xrf.setSleepMode(2, XBEEDTR_PIN) != XRF_OK) {
    delay(1000);
  }
  xrf.init(slaveName);
  xrf.leaveCmndMode();
  delay(100);
  DIAGPRINTLN("XRF slave");

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
  if ((nextWakeUp && ts >= nextWakeUp) || (nextSlaveUpload && ts >= nextSlaveUpload)) {
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

    if (nextSlaveUpload == 0) {
      // We need to ask the master when the next upload is
      doAskNextUpload();
      if (nextSlaveUpload == 0) {
        goto endLoopRetry;
      } else {
        // No need for extra wake up
        nextWakeUp = 0;
      }
    }

    // Is it time for the next upload?
    ts = rtc.now().getEpoch();
    if (ts >= nextSlaveUpload) {
      doUpload();
      if (doneHello) {
        // If we didn't get an NACK then we can ask for "next"
        doAskNextUpload();
      }

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
 * Read all sensor on this device
 */
void doReadSensors()
{
  DeviceDataRecord_t *dr = &dataRecord.dr;
  dr->ts = rtc.now().getEpoch();
  dr->battVolt = getRealBatteryVoltage();
  dr->temp = rtc.getTemperature();

  DIAGPRINT(dr->ts);
  DIAGPRINT(" Battery Voltage: "); DIAGPRINTLN(dr->battVolt);
  // While testing, send it over the XRF air too.
  Serial.print(slaveName); Serial.print(' '); Serial.print(dr->ts);
  Serial.print(" Battery Voltage: "); Serial.println(dr->battVolt);
  DIAGPRINT("RTC Temperature: "); DIAGPRINTLN(dr->temp);
}

/*
 * Send the "hello" command and wait for an "ack"
 */
void doSendHello()
{
  if (xrf.sendData(XRF_MASTER_NAME, "hello") == XRF_OK) {
    doneHello = true;
  }
}

/*
 * Send the "ts" command and wait for the timestamp from the master
 *
 * If the timestamp is received, the RTC is updated with it.
 */
void doSendTs()
{
  char reply[40];

  if (xrf.sendDataAndWaitForReply(XRF_MASTER_NAME, "ts", reply, sizeof(reply)) == XRF_OK) {
    consumeTimestamp(reply);
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
  char reply[40];
  char *ptr;
  char *eptr;

  if (xrf.sendDataAndWaitForReply(XRF_MASTER_NAME, "next", reply, sizeof(reply)) == XRF_OK) {
    ptr = reply;
    uint32_t ts = strtoul(ptr, &eptr, 0);
    if (eptr != ptr) {
      nextSlaveUpload = ts;
      DIAGPRINT("nextUpload: "); DIAGPRINTLN(nextSlaveUpload);
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
  DIAGPRINT("doUpload: "); DIAGPRINTLN(nextSlaveUpload);

  doReadSensors();

  char data[61];
  size_t len = sizeof(dataRecord);
  if (len * 2 > sizeof(data) - 1) {
    DIAGPRINTLN("doUpload: FATAL ERROR: buffer too small");
    return;
  }
  bin2hex(data, sizeof(data), (uint8_t *)&dataRecord, len);

  if (!sendKeyValueAndWaitForAck("data", data)) {
    redoHello();
    return;
  }

  if (uploadInterval) {
    nextSlaveUpload += uploadInterval;
  } else {
    // This will trigger to ask for a new next upload
    nextSlaveUpload = 0;
  }
}

/*
 * Send a key and value and wait for the reply
 */
bool sendKeyValueAndWaitForAck(const char *parm, const char *val)
{
  String line;
  line += parm;
  line += ',';
  line += val;

  return xrf.sendData(XRF_MASTER_NAME, line.c_str()) == XRF_OK;
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
    doneTs = true;
  }
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

void redoHello()
{
  doneHello = false;
  doneTs = false;
  nextSlaveUpload = 0;
  nextWakeUp = rtc.now().getEpoch();
}

/*
 * Update the RTC with the new value
 *
 * If the change is less than a certain minimum then
 * the RTC will not be updated
 */
void setRtc(uint32_t newTs)
{
  uint32_t oldTs = rtc.now().getEpoch();
  int32_t diffTs = abs(newTs - oldTs);
  // Only update the RTC if it differs more than N seconds
  if (diffTs >= 2) {
    // Update the RTC
    rtc.setEpoch(newTs);
    // Make sure we request new upload time, because RTC has changed
    nextSlaveUpload = 0;
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
