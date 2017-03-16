#!/usr/bin/env python

import json
import serial
import struct
import time
import math
import random

SYS_STATE_KEYS = ['m1en', 'm2en', 'm1fwd', 'm2fwd', 'timefail']

def dict2sys(d):
    out = 0
    for k,v in d.items():
        idx = SYS_STATE_KEYS.index(k)
        if idx >= 0 and v:
            out |= 1 << idx
    return out

def sys2dict(num):
    s = bin(num)[2:]
    out = {}
    for idx,val in enumerate(list(reversed(s))[:len(SYS_STATE_KEYS)]):
        out[SYS_STATE_KEYS[idx]] = True if val == '1' else False
    for key in SYS_STATE_KEYS:
        if not key in out:
            out[key] = False
    return out


class Protocol:
    def __init__(self, port='/dev/ttyACM0'):
        self.s = serial.Serial(port, baudrate=115200, timeout=1)

        self.sys_state = None
        self.f1 = None
        self.f2 = None
        self.nsteps1 = None
        self.nsteps2 = None        
        self.p1 = None
        self.p2 = None

        # http://stackoverflow.com/questions/21073086/wait-on-arduino-auto-reset-using-pyserial
        self.s.setDTR(False)
        time.sleep(1)
        self.s.flushInput()
        self.s.setDTR(True)

        # Send a "handshake"
        while not self.s.writable():
            time.sleep(0.01)

        print 'waiting for no reason...'
        time.sleep(2)

    def flush(self):
        while self.s.in_waiting > 0:
            self.s.read()

    def next_cmd(self):
        line = None
        while line not in ('eject', 'load'):
            line = self.s.readline().strip()

        return line

    def lock_read(self):
        cmd = None
        while cmd != 'eject':
            cmd = self.next_cmd()
        return True

    def read(self):
        self.sys_state = self.s.read(1)
        self.f1 = self.s.read(3)
        self.f2 = self.s.read(3)
        try:
            self.nsteps1 = int(self.s.readline())
            self.nsteps2 = int(self.s.readline())
            self.p1 = int(self.s.readline())
            self.p2 = int(self.s.readline())
        except ValueError as err:
            print err
            return False
        return True

    def write(self, sys_state, p1, p2):
        self.s.write("%d\n" % 42)
        state = dict2sys(sys_state)
        self.s.write("%d\n" % state)
        self.s.write("%d\n" % p1)
        self.s.write("%d\n" % p2)
        return True

    def get_f1(self):
        # TODO: think about the sign bit
        # http://stackoverflow.com/questions/3783677/how-to-read-integers-from-a-file-that-are-24bit-and-little-endian-using-python
        return struct.unpack('<i', self.f1 + ('\0' if self.f1[2] < '\x80' else '\xff'))[0]

    def get_f2(self):
        return struct.unpack('<i', self.f2 + ('\0' if self.f2[2] < '\x80' else '\xff'))[0]

    def get_nsteps1(self):
        return self.nsteps1
    def get_nsteps2(self):
        return self.nsteps2

    def get_p1(self):
        return self.p1
    def get_p2(self):
        return self.p2

    def get_sys_state(self):
        return sys2dict(ord(self.sys_state))

    def jsonread(self):
        if not self.read():
            return
        
        key = ['f1', 'f2', 'nsteps1', 'nsteps2', 'p1', 'p2', 'sys_state']
        vals = []
        for k in key:
            vals.append(getattr(self, 'get_%s' % k)())

        return dict(zip(key, vals))

    def printread(self):
        self.read()
        key = ['f1', 'f2', 'nsteps1', 'nsteps2', 'p1', 'p2', 'sys_state']

        for k in key:
            print '%s: %s' % (k, getattr(self, 'get_%s' % k)())

class DummyProtocol(Protocol):
    def __init__(self):
        self.sys_state = {}
        self.f1 = 0
        self.f2 = 0
        self.nsteps1 = 0
        self.nsteps2 = 0        
        self.p1 = 65535
        self.p2 = 65535

        self.eject = True

    def next_cmd(self):
        self.eject = not self.eject
        if not self.eject:
            return 'eject'
        return 'load'

    def read(self):
        # increment
        if self.sys_state.get('m1en'):
            if self.sys_state.get('m1fwd'):
                self.nsteps1 += 65535 / self.p1
            else:
                self.nsteps1 -= 65535 / self.p1

        if self.sys_state.get('m2en'):
            if self.sys_state.get('m2fwd'):
                self.nsteps2 += 65535 / self.p2
            else:
                self.nsteps2 -= 65535 / self.p2

        # Spoof force values
        self.f1 = (math.sin(time.time() / 10) * 10 + random.random()) * 1000
        self.f2 = (math.sin(2 + time.time() / 10) * 10 + random.random()) * 1000

        time.sleep(1.0 / 90)    # 90hz

        return True

    def write(self, sys_state, p1, p2):
        self.sys_state = sys_state
        self.p1 = p1
        self.p2 = p2

    def get_f1(self):
        return self.f1

    def get_f2(self):
        return self.f2

    def get_sys_state(self):
        return self.sys_state
            

if __name__ == '__main__':
    import time
    N = 0

    t0 = None

    p = Protocol()
    p.lock_read()

    while True:
        # identity
        ret = p.jsonread()

        c = p.next_cmd()
        if c == 'load':
            p.write(ret['sys_state'], ret['p1'], ret['p2'])
        else:
            print 'noload'

        p.lock_read()

        N += 1
        if t0 is None:
            t0 = time.time()

        if N % 100 == 0:
            print ret
            print ':: %d: %fHz' % (N, N / (time.time() - t0))
