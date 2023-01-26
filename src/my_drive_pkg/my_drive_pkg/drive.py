#!/usr/bin/env python3

#
# Arduino control node
#   Send left/right wheel velocities
#   Read encoder values (20 Hz)
#   Read battery voltage (0.5 Hz)
#
#   Subscribe:
#       /cmd_vel : (geometry_msgs/Twist)     New velocity

#   Publish:
#       /encoder_vals : (arduino_msgs/EncoderVals)  Current left/right encoder values (ticks)
#       /v_battery : (Float32)  Current battery voltage (volts)

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from my_drive_pkg.arduino import ArduinoSerial
from arduino_msgs.msg import EncoderVals


# Two wheel differential drive node
class MyDrive(Node):
    def __init__(self):
        super().__init__("drive")
        self.get_logger().info("drive node STARTED")

        self.serial_port = self.declare_parameter("serial_port", value="/dev/ttyACM0").value
        self.base_width = self.declare_parameter("base_width", 0.2).value

        self.oldLeft = 0.0  # Previous velocities
        self.oldRight = 0.0

        self.motor_run_left = 0  # - = backwards, 0 = stop, + = forwards
        self.motor_run_right = 0  # - = backwards, 0 = stop, + = forwards
        self.arduino = ArduinoSerial(self.serial_port)
        self.v_bat = 0.0

        # Create subscribers
        self.create_subscription(Twist, "cmd_vel", self.calculate_left_and_right_target, 10)

        # Create publishers
        self.pub_enc_vals = self.create_publisher(EncoderVals, "encoder_vals", 10)
        self.pub_batt_voltage = self.create_publisher(Float32, "v_battery", 10)

        # Timer to read current encoder value
        self.timer = self.create_timer(0.05, self.encoder_check_callback)

        # Battery check timer
        self.timer2 = self.create_timer(2, self.battery_check_callback)
        self.lcd_publish_row_1 = self.create_publisher(String, "/lcd_display/row1", 10)
        self.lcd_publish_row_2 = self.create_publisher(String, "/lcd_display/row2", 10)

    # Read current battery voltage
    def battery_check_callback(self):
        bat_msg = Float32()
        self.arduino.serialSend("<6>")  # Request current battery voltage
        myStr = self.arduino.serialReceive()  # wait for response or timeout
        self.v_bat = float(myStr)
        bat_msg.data = self.v_bat
        self.pub_batt_voltage.publish(bat_msg)  # Publish battery voltage

        lcd_msg = String()
        lcd_msg.data = f"Batt {self.v_bat:.2f}V"
        self.lcd_publish_row_1.publish(lcd_msg)  # Display battery voltage on lcd

    # Read Arduino wheel ticks
    def encoder_check_callback(self):
        self.arduino.serialSend("<1>")  # Request encoder values <left#right>
        rcv_str = self.arduino.serialReceive()  # wait for response or timeout

        enc_msg = EncoderVals()
        s_list = rcv_str.split("#")
        enc_msg.left_motor_enc_val = int(s_list[0])
        enc_msg.right_motor_enc_val = int(s_list[1])
        # enc_msg.stamp = self.get_clock().now().to_msg()
        self.pub_enc_vals.publish(enc_msg)  # Publish encoder total ticks

        # lcd_msg = String()
        # lcd_msg.data = f"L: {enc_msg.left_motor_enc_val} R{enc_msg.right_motor_enc_val}"
        # self.lcd_publish_row_2.publish(lcd_msg)     # Display msg

    def calculate_left_and_right_target(self, msg):  # Convert twist msg to left and right velocities (mm/sec)
        # Assume twist.linear is +/- 0.5 and twist.angular is +/- 1.0 (default teleop key/joy)
        # Wheel targets are in mm/sec
        v_left = 200.0 * msg.linear.x - 2000 * (msg.angular.z * self.base_width / 2.0)
        v_right = 200.0 * msg.linear.x + 2000 * (msg.angular.z * self.base_width / 2.0)
        if (self.oldLeft != v_left) or (self.oldRight != v_right):
            self.oldLeft = v_left
            self.oldRight = v_right
            # self.get_logger().info(f"X: {msg.linear.x}  T: {msg.angular.z}")
            self.change_vel_left(v_left)
            self.change_vel_right(v_right)

            # lcd_msg = String()
            # lcd_msg.data = f"L: {v_left:.0f} R: {v_right:.0f}"
            # self.lcd_publish_row_2.publish(lcd_msg)     # Display msg

    def change_vel_left(self, vel):  # vel (mm/sec)
        if abs(vel) < 0.5:
            self.motor_run_left = 0  # Stop
            self.arduino.serialSend("<2#1#0>")  # Velocity to zero
        else:
            self.arduino.serialSend("<2#1#{:.0f}>".format(vel))  # New velocity (mm/sec)
            if self.motor_run_left == 0:  # Turn motor on continuous
                if vel > 0:
                    self.motor_run_left = 1
                    self.arduino.serialSend("<4#1#1>")  # Run continuous forward
                else:
                    self.motor_run_left = -1
                    self.arduino.serialSend("<4#1#-1>")  # Run continuous backward
            else:
                if self.motor_run_left == 1:  # Already running forward
                    if vel < 0:
                        self.motor_run_left = -1
                        self.arduino.serialSend("<4#1#-1>")  # Change to run backward
                else:  # Already running backward
                    if vel > 0:  # Change to run forward
                        self.motor_run_left = 1
                        self.arduino.serialSend("<4#1#1>")  # Change to run forward

    def change_vel_right(self, vel):  # vel (mm/sec)
        if abs(vel) < 0.5:
            self.motor_run_right = 0  # Stop
            self.arduino.serialSend("<2#2#0>")
        else:
            self.arduino.serialSend("<2#2#{:.0f}>".format(vel))  # New velocity (mm/sec)
            if self.motor_run_right == 0:  # Turn motor on continuous
                if vel > 0:
                    self.motor_run_right = 1
                    self.arduino.serialSend("<4#2#1>")  # Run continuous forward
                else:
                    self.motor_run_right = -1
                    self.arduino.serialSend("<4#2#-1>")  # Run continuous backward
            else:
                if self.motor_run_right == 1:  # Already running forward
                    if vel < 0:  # Change to run backward
                        self.motor_run_right = -1
                        self.arduino.serialSend("<4#2#-1>")
                else:  # Already running backward
                    if vel > 0:  # Change to run forward
                        self.motor_run_right = 1
                        self.arduino.serialSend("<4#2#1>")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MyDrive()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.arduino.close()
        node.destroy_node()
        node.get_logger().info("drive node has shutdown")
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
