#!/usr/bin/env python3

#
# Arduino communications
#
#   serial_send((bytes))    Send text to arduino
#   serial_receive()        Wait for receive text (100 msec timeout)

import serial
import time


class ArduinoSerial:
    def __init__(self, port):
        # ls -l /dev | grep ACM to identify serial port of the arduino
        self.arduino = serial.Serial(port, 115200, timeout=0.1)
        self.arduino.setDTR(False)
        time.sleep(1)
        self.arduino.flushInput()
        self.arduino.setDTR(True)
        time.sleep(2)
        self.lastTime = 0.0
        self.previousMillis = 0.0
        self.cmd_wait = False

    # Send command to serial port
    def serialSend(self, command):
        self.arduino.write(bytes(command, encoding="utf8"))  # Arduino
        self.lastTime = time.time()
        self.cmd_wait = True

    # Receive serial port bytes, return string
    def serialReceive(self):
        if self.cmd_wait:       # Set if command returns response
            try:
                startMarker = 60  # '<'
                endMarker = 62  # '>'

                getSerialValue = bytes()
                byteCount = -1  # last increment will be one too many

                x = self.arduino.read()     # Read next byte or timeout
                if x == b'':    # Timeout
                    myStr = ""
                else:
                    # wait for the start character
                    while ord(x) != startMarker:
                        x = self.arduino.read()

                    # save data until the end marker is found
                    while ord(x) != endMarker:
                        if ord(x) != startMarker:
                            getSerialValue = getSerialValue + x
                            byteCount += 1
                            if byteCount > 100:
                                self.cmd_wait = False
                                return ""
                        x = self.arduino.read()
                    myStr = getSerialValue.decode("ascii", errors="replace")

            except ValueError:
                pass

            self.arduino.flushInput()
            self.cmd_wait = False

            return myStr

    def close(self):
        self.arduino.close()
