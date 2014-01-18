/*
 * A simple sketch that sends some (sensor) data to another
 * XRF device.
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
const char *myName = "dev1";

XRF xrf(Serial, myName, panId);

// This is the address of the other XRF device.
const char *receiver = "dev2";

void setup()
{
  Wire.begin();
  rtc.begin();

  Serial.begin(9600);
  xrf.init();
}

float temp;
void sendData()
{
  char buffer[60];

  // Prepare a packet
  dtostrf(temp, 0, 2, buffer);

  // Send the packet and wait for an answer.
  uint8_t status = xrf.sendData(receiver, buffer);
  if (status != XRF_OK) {
    // What now? It was already retried a few times.
  }
}

void loop()
{
  temp = rtc.getTemperature();
  sendData();
  delay(10000);
}
