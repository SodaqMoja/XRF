/*
 * A simple sketch that receives some (sensor) data from
 * another XRF device.
 */
#include <Sodaq_DS3231.h>
#include <Wire.h>
#include <XRF.h>

// These two pins are for SODAQ Moja
#define XBEEDTR_PIN     7
#define XBEECTS_PIN     8

//#########       variables      #############
// Default panID is 0x5AA5. Using that makes it simple to plug in a monitor
// For more "security via obscurity" you could pick another value.
const int panId = 0x5AA5;
// This is the name that we listen to when using xrf.receiveMyData
const char *myName = "dev2";

XRF xrf(Serial, panId, myName);

uint32_t getNow();

void setup()
{
  Wire.begin();
  rtc.begin();

  // Install the callback function that will instruct the XRf library
  // to send our RTC in the ACK packets
  xrf.setGetNowFunc(getNow);

  Serial.begin(9600);
  // We want this to succeed
  while (xrf.setSleepMode(1, XBEEDTR_PIN) != XRF_OK) {
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

/*
 * Do something useful with the received data
 */
void processData(const char *buffer)
{
  char *eptr;
  double val;
  // Expecting a floating point number in ASCII representation
  val = strtod(buffer, &eptr);
  if (eptr != buffer) {
    temp = val;
  }
}

void loop()
{
  char buffer[60];
  char source[16];
  if (xrf.receiveData(source, sizeof(source), buffer, sizeof(buffer)) == XRF_OK) {
    processData(buffer);
  }
}

uint32_t getNow()
{
  return rtc.now().getEpoch();
}
