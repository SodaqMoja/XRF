This is an Arduino library to send and receive data
across a XRF radio network.

The data packets are ASCII lines, terminated with an
end of line (which is configurable). The packets have
a CRC16 checksum at the end. We're using _crc_xmodem_update
from AVR libc for that. Not perfect, but hopefully
sufficient.

There is an example application implemented in two sketches,
one is a "master" device and the other sketch is a "slave".
There can be just one master, and up to 10 slaves. The slaves
send packets with a "M," prefix which is recognized by the
master. The master will respond with a packet that has the
slave device's name as the prefix.

The example application has some logic to synchronize the time
between all devices. The time of the master is leading.
