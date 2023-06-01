"""RVIZ click to 2D."""

# !/usr/bin/env python3
# Convert initial_pose and goal_pose clicks to 2d
#
#   Subscribe:
#       /initial_pose : (geometry_msgs/PoseWithCovarianceStamped)
#       /goal_pose : (geometry_msgs/PoseStamped)
#
#   Publish:
#       /initial_2d : (geometry_msgs/PoseStamped)
#       /goal_2d : (geometry_msgs/PoseStamped)
#

import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Click_to_2d(Node):
    """
    Convert initial pose/goal to 2d.

    Subscribe to rviz pose topics
    Publish 2d poses
    """

    def __init__(self):
        """Init class."""
        super().__init__('pose_to_2d')
        self.get_logger().info('pose_to_2d node STARTED')

        # Create publisher/subscriber topics
        self.pub_initial_2d = self.create_publisher(PoseStamped, 'initial_2d', 10)
        self.pub_goal_2d = self.create_publisher(PoseStamped, 'goal_2d', 10)
        self.create_subscription(PoseWithCovarianceStamped, 'initial_pose', self.handle_initial_pose, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.handle_goal_pose, 10)

        self.lcd_publish_row_2 = self.create_publisher(String, '/lcd_display/row2', 10)

    def handle_goal_pose(self, msg):
        """Goal position clicked."""
        goalPose = PoseStamped()
        self.get_logger().info(f'goal_pose: {msg.pose}')

        goalPose.header.frame_id = 'map'
        goalPose.header.stamp = msg.header.stamp
        goalPose.pose.position.x = msg.pose.position.x
        goalPose.pose.position.y = msg.pose.position.y
        goalPose.pose.position.z = 0.0

        roll, pitch, yaw = self.euler_from_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        goalPose.pose.orientation.x = 0.0
        goalPose.pose.orientation.y = 0.0
        goalPose.pose.orientation.z = yaw
        goalPose.pose.orientation.w = 0.0
        self.pub_goal_2d.publish(goalPose)

        lcd_msg = String()
        lcd_msg.data = f'Goal {goalPose.pose.position.x:.2f},{goalPose.pose.position.y:.2f}'
        self.lcd_publish_row_2.publish(lcd_msg)

    def handle_initial_pose(self, msg):
        """Click initial position."""
        initialPose = PoseStamped()
        self.get_logger().info(f'initial_pose: {msg.pose.pose}')

        initialPose.header.frame_id = 'map'
        initialPose.header.stamp = msg.header.stamp
        initialPose.pose.position.x = msg.pose.pose.position.x
        initialPose.pose.position.y = msg.pose.pose.position.y
        initialPose.pose.position.z = 0.0

        roll, pitch, yaw = self.euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        initialPose.pose.orientation.x = 0.0
        initialPose.pose.orientation.y = 0.0
        initialPose.pose.orientation.z = yaw
        initialPose.pose.orientation.w = 0.0
        self.pub_initial_2d.publish(initialPose)
        self.get_logger().info(f'initial_pose: {initialPose}')

        lcd_msg = String()
        lcd_msg.data = f'Init {initialPose.pose.position.x:.2f},{initialPose.pose.position.y:.2f}'
        self.lcd_publish_row_2.publish(lcd_msg)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw).

        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    try:
        click_2d = Click_to_2d()
        rclpy.spin(click_2d)

    except KeyboardInterrupt:
        pass

    finally:
        click_2d.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
