#!/usr/bin/env python

import sys
import json
import time
import serial
import threading
from SimpleHTTPServer import SimpleHTTPRequestHandler
from BaseHTTPServer import HTTPServer
from wsserve import WebsocketServer
import protocol

UI_PORT = 8080
WS_PORT = 9999

dummy = sys.argv[1] == 'dummy' if len(sys.argv) > 1 else False

class SyncedServer(WebsocketServer):
    def onconnect(self, sender):
        # Connect to the Arduino
        if dummy:
            print 'dummy'
            chip = protocol.DummyProtocol()
        else:
            try:
                chip = protocol.Protocol()
            except serial.SerialException:
                print 'cannot connect: falling back to dummy'
                chip = protocol.DummyProtocol()

        ticks = 0
        t0 = None
        start = True

        while True:
            if t0 is None:
                t0 = time.time()

            # Read from chip
            #print 'chipread'
            chip.lock_read()
            chip_info = chip.jsonread()
            if chip_info is None:
                print 'r/fail'
                continue

            if start:
                print chip_info
                start = False

            # Send to JS
            #print 'jssend'
            sender.send_message(json.dumps(chip_info))

            # Get spec back from JS
            #print 'jsread'
            ret = json.loads(sender.read_next_message())

            cmd = chip.next_cmd()
            if cmd == 'load':
                # Send back to the chip
                #print 'chipsend'
                chip.write(ret['sys_state'],
                           int(ret['p1']) if ret['p1'] else 0,
                           int(ret['p2']) if ret['p2'] else 0)
            else:
                print 'oops', cmd

            ticks += 1

            if ticks % 100 == 0:
                t = time.time()
                print ':: %d: %f Hz' % (ticks, 100 / (t - t0))
                t0 = t

            #time.sleep(1)


ws_server = SyncedServer(WS_PORT)

ui_server = HTTPServer(('', UI_PORT), SimpleHTTPRequestHandler)
thread = threading.Thread(target = ui_server.serve_forever)
thread.daemon = True
thread.start()

print 'http://localhost:%i/www/' % UI_PORT

ws_server.serve_forever()
