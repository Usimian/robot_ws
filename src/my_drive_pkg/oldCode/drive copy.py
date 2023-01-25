#!/usr/bin/env python3

#
# Differential drive to arduino
#   Send left/right wheel velocities
#
#   Subscribe:
#       /lwheel_vtarget : (std_msgs/Float32)    Left wheel velocity (mm/sec)
#       /rwheel_vtarget : (std_msgs/Float32)    Right wheel velocity (mm/sec)
#
#   Publish:
#       /serial_cmd : (std_msgs/String)     Arduino command string

import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
from std_msgs.msg import String


class MyDrive(Node):
    def __init__(self):
        super().__init__("drive")
        self.get_logger().info("drive node STARTED")
        self.motor_run_left = 0      # - = backwards, 0 = stop, + = forwards
        self.motor_run_right = 0      # - = backwards, 0 = stop, + = forwards
        # Create subscribers
        self.create_subscription(Float32, "lwheel_vtarget", self.change_vel_left, 10)
        self.create_subscription(Float32, "rwheel_vtarget", self.change_vel_right, 10)
        # Create publishers
        self.pub_serial_cmd_NR = self.create_publisher(String, "serial_cmd_no_response", 10)
        self.pub_serial_cmd = self.create_publisher(String, "serial_cmd", 10)

    def change_vel_left(self, msg):
        # self.get_logger().info(f"lwheel_vtarget: {msg.data}")
        cmd = String()
        if abs(msg.data) < 0.5:
            self.motor_run_left = 0  # Stop
            cmd.data = "<2#1#0>"  # Velocity to zero
            self.pub_serial_cmd_NR.publish(cmd)
        else:
            cmd.data = "<2#1#{:.0f}>".format(msg.data)  # New velocity (mm/sec)
            self.pub_serial_cmd_NR.publish(cmd)
            if self.motor_run_left == 0:  # Turn motor on continuous
                if msg.data > 0:
                    cmd.data = "<4#1#1>"  # run continuous forward
                    self.motor_run_left = 1
                    self.pub_serial_cmd_NR.publish(cmd)
                else:
                    cmd.data = "<4#1#-1>"  # run continuous backward
                    self.motor_run_left = -1
                    self.pub_serial_cmd_NR.publish(cmd)
            else:
                if self.motor_run_left == 1:  # Already running forward
                    if msg.data < 0:  # Change to run backward
                        cmd.data = "<4#1#-1>"
                        self.motor_run_left = -1
                        self.pub_serial_cmd_NR.publish(cmd)
                else:  # Already running backward
                    if msg.data > 0:  # Change to run forward
                        cmd.data = "<4#1#1>"
                        self.motor_run_left = 1
                        self.pub_serial_cmd_NR.publish(cmd)
            # self.get_logger().info(f"left wheel: {cmd}")


    def change_vel_right(self, msg):
        # self.get_logger().info(f"rwheel_vtarget: {msg.data}")
        cmd = String()
        if abs(msg.data) < 0.5:
            self.motor_run_right = 0  # Stop
            cmd.data = "<2#2#0>"  # Velocity to zero
            self.pub_serial_cmd_NR.publish(cmd)
        else:
            cmd.data = "<2#2#{:.0f}>".format(msg.data)  # New velocity
            self.pub_serial_cmd_NR.publish(cmd)
            if self.motor_run_right == 0:  # Turn motor on continuous
                if msg.data > 0:
                    cmd.data = "<4#2#1>"  # run continuous forward
                    self.motor_run_right = 1
                    self.pub_serial_cmd_NR.publish(cmd)
                else:
                    cmd.data = "<4#2#-1>"  # run continuous backward
                    self.motor_run_right = -1
                    self.pub_serial_cmd_NR.publish(cmd)
            else:
                if self.motor_run_right == 1:  # Already running forward
                    if msg.data < 0:  # Change to run backward
                        cmd.data = "<4#2#-1>"
                        self.motor_run_right = -1
                        self.pub_serial_cmd_NR.publish(cmd)
                else:  # Already running backward
                    if msg.data > 0:  # Change to run forward
                        cmd.data = "<4#2#1>"
                        self.motor_run_right = 1
                        self.pub_serial_cmd_NR.publish(cmd)
            # self.get_logger().info(f"right wheel: {cmd}")


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
        node.destroy_node()
        node.get_logger().info("drive node has shutdown")
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
