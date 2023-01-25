#!/usr/bin/env python3

#
# Convert Twist message to left/right wheel velocities
#
#   Subscribe:
#       /cmd_vel : (geometry_msgs/Twist)     New velocity
#
#   Publish:
#       /lwheel_vtarget : (std_msgs/Float32)    Left wheel velocity (mm/sec)
#       /rwheel_vtarget : (std_msgs/Float32)    Right wheel velocity (mm/sec)

import sys
import rclpy

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class TwistToMotors(Node):
    # twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack

    def __init__(self):
        super().__init__("twist_to_motors")
        self.get_logger().info("twist_to_motors node STARTED")

        self.base_width = self.declare_parameter("base_width", 0.2).value
        self.oldLeft = 0.0
        self.oldRight = 0.0

        # Create publisher/subscriber topics
        self.pub_lmotor = self.create_publisher(Float32, "lwheel_vtarget", 10)
        self.pub_rmotor = self.create_publisher(Float32, "rwheel_vtarget", 10)
        self.create_subscription(Twist, "cmd_vel", self.calculate_left_and_right_target, 10)

    def calculate_left_and_right_target(self, msg):

        left = Float32()
        right = Float32()

        # Assume twist.linear is +/- 0.5 and twist.angular is +/- 1.0 (default teleop key/joy)
        # Wheel targets are in mm/sec
        left.data = 200.0 * msg.linear.x - 2000 * (msg.angular.z * self.base_width / 2.0)
        right.data = 200.0 * msg.linear.x + 2000 * (msg.angular.z * self.base_width / 2.0)
        if (self.oldLeft != left.data) or (self.oldRight != right.data):
            self.oldLeft = left.data
            self.oldRight = right.data
            # self.get_logger().info(f"X: {msg.linear.x}  T: {msg.angular.z}")
            self.pub_lmotor.publish(left)
            self.pub_rmotor.publish(right)


def main(args=None):
    rclpy.init(args=args)
    try:
        twist_to_motors = TwistToMotors()
        rclpy.spin(twist_to_motors)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        twist_to_motors.destroy_node()
        twist_to_motors.get_logger().info("twist_to_motors node has shutdown")
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
