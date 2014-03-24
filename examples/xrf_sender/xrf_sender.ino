/*
 * A simple sketch that sends some (sensor) data to another
 * XRF device.
 */
#include <Sodaq_DS3231.h>
#include <Wire.h>
#include <XRF.h>

#include "pindefs.h"
#include "Diag.h"

//#########       variables      #############
// Default panID is 0x5AA5. Using that makes it simple to plug in a monitor
// For more "security via obscurity" you could pick another value.
const int panId = 0x5AA5;
// This is the name that we listen to when using xrf.receiveMyData
const char *myName = "dev1";

XRF xrf(Serial, panId, myName, 2, 500);

// This is the address of the other XRF device.
const char *receiver = "dev2";

void setRtc(uint32_t ts);

void setup()
{
  Wire.begin();
  rtc.begin();

  // Install the callback function that will instruct the XRF
  // library to update the RTC with the timestamp in the incoming
  // ACK packets.
  xrf.setSetNowFunc(setRtc);

  Serial.begin(9600);
#ifdef ENABLE_DIAG
  diagport.begin(9600);
  xrf.setDiag(diagport);
#endif
  DIAGPRINTLN(F("xrf_sender"));

  // Sleep mode is important. We must set it before anything else.
  // We want this to succeed
  while (xrf.setSleepMode(2, XBEEDTR_PIN) != XRF_OK) {
  }
  // DataRate 2 is 38.4k.
  // DataRate 4 is 100k.
  // DataRate 5 is 50k.
  while (xrf.setDataRate(2) != XRF_OK) {
  }
  xrf.doApplyChanges();
  xrf.init();
  xrf.leaveCmndMode();
}

float temp;
void sendData()
{
  char buffer[60];

  // Prepare a packet
  dtostrf(temp, 0, 2, buffer);

  xrf.wakeUp();
  delay(100);                 // FIXME How much is needed, if any?
  Serial.print(myName); Serial.println(F(" start wakeup"));
  DIAGPRINT(myName); DIAGPRINTLN(" start wakeup");

  // Send the packet and wait for an answer.
  uint8_t status = xrf.sendData(receiver, buffer);
  if (status != XRF_OK) {
    // What now? It was already retried a few times.
  }

  strcpy(buffer, "445095000:17.7578:3.27");
  status = xrf.sendData(receiver, buffer);
  if (status != XRF_OK) {
    // What now? It was already retried a few times.
  }

  xrf.sleep();
}

void loop()
{
  temp = rtc.getTemperature();
  sendData();
  delay(2000);
}

/*
 * Update the RTC
 *
 * This function is called from the XRF library when an ACK
 * is received.
 */
void setRtc(uint32_t newTs)
{
  DIAGPRINTLN(F("setRtc"));
  uint32_t oldTs = rtc.now().getEpoch();
  int32_t diffTs = abs(newTs - oldTs);
  if (diffTs > 2) {
    // The timestamp differs too much. Adjust the RTC.
    DIAGPRINTLN(F("setRtc is updating the RTC"));
    rtc.setEpoch(newTs);
  }
}
