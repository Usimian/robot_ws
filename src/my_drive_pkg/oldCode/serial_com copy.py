#!/usr/bin/env python3

#
# Arduino communications
#
#   Subscribe:
#       /serial_cmd : (std_msgs/String)     Arduino command string to send
#       /serial_cmd_no_response : (std_msgs/String)     Arduino command string to send (no response published)
#
#   Publish:
#       /serial_response : (std_msgs/String)     Arduino response string

import serial
import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String


class MySerial(Node):
    def __init__(self):
        super().__init__("serial_com")
        self.get_logger().info("serial_com node STARTED")
        self.my_serial = ArduinoSerial("/dev/ttyACM0")
        self.create_subscription(String, "serial_cmd", self.send_serial_cmd, 10)
        self.create_subscription(String, "serial_cmd_no_response", self.send_serial_cmd_no_response, 10)
        self.pub_serial_response = self.create_publisher(String, "serial_response", 10)

    def send_serial_cmd(self, msg):  # msg is formatted command string, publish response
        # then = time.time()
        myStrCmd = msg.data
        self.my_serial.serialSend(myStrCmd)

        self.myStrRcv = String()
        myStr = self.my_serial.serialReceive()  # wait for response or timeout
        self.myStrRcv.data = myStr
        self.pub_serial_response.publish(self.myStrRcv)
        # now = time.time()
        # self.get_logger().info(f"Delta time {myStrCmd}: {now - then}")
        # self.get_logger().info(f"send_serial_cmd: {myStrCmd} : {self.myStrRcv.data}")

    def send_serial_cmd_no_response(self, msg):  # msg is formatted command string
        myStrCmd = msg.data
        self.my_serial.serialSend(myStrCmd)


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
        if self.cmd_wait:
            try:
                startMarker = 60  # '<'
                endMarker = 62  # '>'

                getSerialValue = bytes()
                byteCount = -1  # last increment will be one too many

                x = self.arduino.read() # Read next byte or timeout
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


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MySerial()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        node.get_logger().info("serial_com node has shutdown")
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
