#!/usr/bin/env python
'''
An example HUB program to receive packets from SODAQ XRF nodes.

The class HubHandler is doing the actual work. Have a look at

'''

import sys
import os
import argparse
import time
import threading
import xrf

EXITCHARCTER = '\x1d'   # GS/CTRL+]

def key_description(character):
    """generate a readable description for a key"""
    ascii_code = ord(character)
    if ascii_code < 32:
        return 'Ctrl+%c' % (ord('@') + ascii_code)
    else:
        return repr(character)

if sys.version_info >= (3, 0):
    def cnvToCharBuf(b):
        return b.decode('latin1')
else:
    def cnvToCharBuf(b):
        return b

# first choose a platform dependent way to read single characters from the console
console = None

if os.name == 'nt':
    import msvcrt
    class Console(object):
        def __init__(self):
            pass

        def setup(self):
            pass    # Do nothing for 'nt'

        def cleanup(self):
            pass    # Do nothing for 'nt'

        def getkey(self):
            while True:
                z = msvcrt.getch()
                if z == '\0' or z == '\xe0':    # functions keys, ignore
                    msvcrt.getch()
                else:
                    if z == '\r':
                        return '\n'
                    return z

    console = Console()

elif os.name == 'posix':
    import termios, sys, os
    class Console(object):
        def __init__(self):
            self.fd = sys.stdin.fileno()

        def setup(self):
            self.old = termios.tcgetattr(self.fd)
            new = termios.tcgetattr(self.fd)
            new[3] = new[3] & ~termios.ICANON & ~termios.ECHO & ~termios.ISIG
            new[6][termios.VMIN] = 1
            new[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, new)

        def getkey(self):
            c = os.read(self.fd, 1)
            return c

        def cleanup(self):
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old)

    console = Console()

    def cleanup_console():
        console.cleanup()

    console.setup()
    sys.exitfunc = cleanup_console      # terminal modes have to be restored on exit...

else:
    raise NotImplementedError("Sorry no implementation for your platform (%s) available." % sys.platform)

class HubHandler(object):
    '''
    This handler is in fact the most important part of the program.
    It continuously reads new packets and take action on them.
    See "cmds" below which forms the list of recognized commands.
    '''
    def __init__(self, xrf, args):
        self._xrf = xrf
        self._args = args
        self._alive = False

    @property
    def alive(self):
        return self._alive
    @alive.setter
    def alive(self, x):
        self._alive = x

    def packet_reader(self):
        '''
        This is "main loop" of the packet handler. It reads a packet
        looks at the command field and it takes action.
        '''
        cmds = {
                'data': self._handleData,
                'ts': self._handleTs,
                }
        # Keep reading packets
        while self.alive:
            status, source, data = self._xrf.receiveData(4000)
            #self._diagPrintLn("new packet: status=%(status)d source=%(source)s data='%(data)s'" % vars())
            if status != xrf.XRF.XRF_OK:
                continue
            #self._diagPrintLn("new packet: source=%(source)s data='%(data)s'" % vars())
            try:
                # We expect at least one fields
                if data.find(',') >= 0:
                    cmd, rest = data.split(',', 1)
                else:
                    cmd = data
                    rest = None
                self._diagPrintLn("new packet: source=%(source)s cmd=%(cmd)s rest='%(rest)s'" % vars())
                if cmd in cmds:
                    cmds[cmd](source, rest)
            except:
                # Just skip it.
                continue

    def _handleData(self, source, data):
        self._diagPrintLn("_handleData: data='%(data)s'" % vars())

        # TODO Do something with the received data
        # Let's assume that the fields are comma separated
        # The other of fields is determined by the sender.
        v = data.split(',')

    def _handleTs(self, source, data):
        self._diagPrintLn("_handleTs: data='%(data)s'" % vars())
        # Send our timestamp
        ts = int(time.time())
        self._xrf.sendData(source, str(ts))

    def _diagPrintLn(self, txt):
        if not self._args.verbose:
            return
        sys.stdout.write(txt + '\n')
        sys.stdout.flush()

def mainLoop(args, hub):
    '''
    This is a helper function to start a thread with HubHandler, and also
    a loop that checks the console for a clean exit (the CTRL-] key).
    '''
    try:
        sys.stderr.write('Entering main loop. To quit please type %s\n' % (
            key_description(EXITCHARCTER),
            ))

        handler = HubHandler(hub, args)
        handler.alive = True
        receiver_thread = threading.Thread(target=handler.packet_reader)
        receiver_thread.setDaemon(True)
        receiver_thread.start()

        while True:
            try:
                b = console.getkey()
            except KeyboardInterrupt:
                b = serial.to_bytes([3])
            c = cnvToCharBuf(b)
            if c == EXITCHARCTER:
                handler.alive = False
                break
        sys.stdout.write("waiting for packet_reader to finish\n")
        sys.stdout.flush()
        receiver_thread.join()
    except:
        raise

def main():
    parser = argparse.ArgumentParser(description='Listen to XRF for data from weather station')
    parser.add_argument('-v', '--verbose', action='store_true', default=False,
                        help='be verbose')
    parser.add_argument('name', default='HUB',
                        help='the name of this device')
    parser.add_argument('ttyport',
                        help='the tty port (e.g. /dev/ttyUSB0')
    args = parser.parse_args()
    #print(args)

    # Create and initialize the XRF device
    hub = xrf.XRF(args.ttyport)
    #while hub.setDataRate(2) != xrf.XRF.XRF_OK:
    #    pass
    #hub.doApplyChanges()
    hub.init(args.name, 0x5AA5)

    mainLoop(args, hub)

if __name__ == '__main__':
    main()
