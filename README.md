XRF Library
===========

This is an Arduino library to send and receive data
across a XRF radio network.

The data packets are sent as ASCII lines, terminated with an
end of line (which is configurable). Each packet (each line)
is assumed to have several fields, separated by a comma (which
is configurable). The first field is the destination name, the
second field is the source name. The last field is a CRC16
checksum. We're using the "xmodem" CRC algorithm from AVR libc


Example(s)
----------

There is an example application implemented in two sketches,
one is a "master" device and the other sketch is a "slave".
There can be just one master, and up to 10 slaves. The slaves
send packets with a "M," prefix which is recognized by the
master. The master will respond with a packet that has the
slave device's name as the prefix. See examples xrf_master
and xrf_slave.

This example application has some logic to synchronize the time
between all devices. The time of the master is leading.

Another example (also in two sketches) shows just a simple
"sender" and a "receiver". See xrf_sender and xrf_receiver.

Implementation details
----------------------

There is a C++ class, named XRF. that must instantiated. You
can create more than one if your Arduino environment has room
for it.

An XRF object needs a reference to a Stream object to operate.
Mostly this means that Serial is used. This is also the case
when a XRFbee is inserted in a bee slot.

The configuration of the xrf device goes in a few (optional)
steps.
* the creation of the XRF object
* calling the XRF::init() function
* calling the XRF::config() function
* calling zero or more "setter" functions

(( Perhaps, at some point in time, we may have to think this over. ))


```
const int panId = 0x5AA5;
const char *myName = "dev1";
XRF xrf(Serial, panId, myName);
void setup()
{
  xrf.init();
  xrf.leaveCmndMode();
}
void loop()
{
  // ...
  xrf.sendData("dest", "hello, world");
}
```

Most important functions
------------------------
* XRF::init
* XRF::config
* XRF::sendData
* XRF::receiveData
* XRF::sendDataAndWaitForReply
