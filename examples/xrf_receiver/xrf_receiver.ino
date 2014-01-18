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

XRF xrf(Serial, myName, panId);

void setup()
{
  Wire.begin();
  rtc.begin();

  Serial.begin(9600);
  xrf.init();
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
