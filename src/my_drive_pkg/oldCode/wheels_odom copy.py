#
# Compute and publish odometry from arduino wheel ticks & print battery voltage on lcd
#   Publishes odom -> base_link
#
#   Subscribe:
#       /initial_2d : The initial position and orientation of the robot.
#                   (geometry_msgs/PoseStamped)
#
#   Publish:
#       /odom_data_euler : Position and velocity estimate.
#           The orientation.z variable is an Euler angle representing the yaw angle.
#           (nav_msgs/Odometry)
#
#       /odom : Raw position and velocity estimate.
#           The orientation is in quaternion format.
#           (nav_msgs/Odometry)
#
#       /serial_cmd : command to Arduino
#           (string)
#       /serial_response : response from Arduino
#           (string)
#
#   Parameters:
#       odom_frame  (string)
#       child_frame (string)

import sys
from time import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf_transformations
from nav_msgs.msg import Odometry
from math import sin, cos, asin, pi, isnan

# Robot physical constants
TICKS_PER_REVOLUTION = 408.0  # For reference purposes.
WHEEL_RADIUS_METERS = 0.030  # Wheel radius in meters
WHEEL_BASE_METERS = 0.170  # Center of left tire to center of right tire
TICKS_PER_METER = TICKS_PER_REVOLUTION / (WHEEL_RADIUS_METERS * 2 * pi)


class WheelsOdom(Node):
    def __init__(self):
        super().__init__("wheels_odom")
        self.get_logger().info("wheels_odom node STARTED")

        # Parameters
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.child_frame = self.declare_parameter("child_frame", "base_footprint").value

        # Initial pose
        self.initialX = 0.0
        self.initialY = 0.0
        self.initialTheta = 0.00000000001

        # Distance both wheels have traveled
        self.distanceLeft = 0.0  # meters
        self.distanceRight = 0.0  # meters
        self.lastCountL = 0
        self.lastCountR = 0

        # Flag to see if initial pose has been received
        self.initialPoseRecieved = True

        # Odometry messages
        self.odomNew = Odometry()
        self.odomOld = Odometry()

        # Parameters
        self.odomNew.header.frame_id = self.odom_frame
        self.odomNew.pose.pose.position.z = 0.0
        self.odomNew.pose.pose.orientation.x = 0.0
        self.odomNew.pose.pose.orientation.y = 0.0
        self.odomNew.twist.twist.linear.x = 0.0
        self.odomNew.twist.twist.linear.y = 0.0
        self.odomNew.twist.twist.linear.z = 0.0
        self.odomNew.twist.twist.angular.x = 0.0
        self.odomNew.twist.twist.angular.y = 0.0
        self.odomNew.twist.twist.angular.z = 0.0
        self.odomOld.pose.pose.position.x = self.initialX
        self.odomOld.pose.pose.position.y = self.initialY
        self.odomOld.pose.pose.orientation.z = self.initialTheta

        self.vBatt = 0.0

        # Subscriber to initial_2d (from initial_pose)
        self.create_subscription(PoseStamped, "initial_2d", self.set_initial_2d, 10)

        # Publisher of simple odom message where orientation.z is an euler angle
        self.odom_data_pub_euler = self.create_publisher(Odometry, "odom_data_euler", 10)

        # Publisher of full odom message where orientation is quaternion
        self.odom_data_pub_quat = self.create_publisher(Odometry, "odom", 10)

        # To get wheel ticks
        self.pub_serial_cmd = self.create_publisher(String, "serial_cmd", 10)
        self.create_subscription(String, "serial_response", self.serial_response_callback, 10)

        # Odometry update timer
        self.timer1 = self.create_timer(0.05, self.odometry_update_callback)

        # Battery check
        self.timer2 = self.create_timer(1, self.battery_check_callback)
        self.lcd_publish_row_1 = self.create_publisher(String, "/lcd_display/row1", 10)

    # Response from Arduino wheel ticks request
    def serial_response_callback(self, msg):
        s = msg.data
        s_list = s.split("#")
        self.vBatt = float(s_list[0])
        leftCount = int(s_list[1])
        rightCount = int(s_list[2])

        if leftCount != 0 and self.lastCountL != 0:
            leftTicks = leftCount - self.lastCountL
            self.distanceLeft = leftTicks / TICKS_PER_METER
            # if self.lastCountL != leftCount:
            #     self.get_logger().info(f"lastCountL: {self.lastCountL}")
            #     self.get_logger().info(f"distanceLeft: {self.distanceLeft}")
        self.lastCountL = leftCount

        if rightCount != 0 and self.lastCountR != 0:
            rightTicks = rightCount - self.lastCountR
            self.distanceRight = rightTicks / TICKS_PER_METER
            # if self.lastCountR != rightCount:
            #     self.get_logger().info(f"lastCountR: {self.lastCountR}")
            #     self.get_logger().info(f"distanceRight: {self.distanceRight}")
        self.lastCountR = rightCount

    def battery_check_callback(self):
        cmd = String()
        cmd.data = f"Batt: {self.vBatt:.2f}V"  # Current battery voltage
        self.lcd_publish_row_1.publish(cmd)

    #  Get wheel ticks, then compute and publish new odom
    def odometry_update_callback(self):
        if self.initialPoseRecieved:
            cmd = String()
            cmd.data = "<1>"  # Req current wheel encoder counts, returns <vBatt, left, right>
            self.pub_serial_cmd.publish(cmd)

            x = self.odomOld.pose.pose.position.x
            # y = self.odomOld.pose.pose.position.y
            # t = self.odomOld.pose.pose.orientation.z

            self.update_odom()
            self.publish_quat()

            if (
                (x != self.odomOld.pose.pose.position.x)
                # or (y != self.odomOld.pose.pose.position.y)
                # or (t != self.odomOld.pose.pose.orientation.z)
            ):
                self.get_logger().info(
                    f"X:{self.odomOld.pose.pose.position.x:.3f}"
                    # f"X:{self.odomOld.pose.pose.position.x:.3f} Y:{self.odomOld.pose.pose.position.y:.3f} T:{self.odomOld.pose.pose.orientation.z:.3f}"
                    # f"Xa:{self.odomNew.twist.twist.linear.x:.3f} Ya:{self.odomNew.twist.twist.linear.y:.3f} Ta:{self.odomNew.twist.twist.angular.z:.3f}"
                )

    #  Get initial_2d message from either Rviz clicks or a manual pose publisher
    def set_initial_2d(self, msg):
        self.odomOld.pose.pose.position.x = msg.pose.position.x
        self.odomOld.pose.pose.position.y = msg.pose.position.y
        self.odomOld.pose.pose.orientation.z = msg.pose.orientation.z
        self.initialPoseRecieved = True
        self.get_logger().info("initial_2d received")

    # Publish an Odometry message in quaternion format
    def publish_quat(self):
        q = tf_transformations.quaternion_from_euler(0, 0, self.odomNew.pose.pose.orientation.z)

        self.quatOdom = Odometry()

        self.quatOdom.header.stamp = self.odomNew.header.stamp
        self.quatOdom.header.frame_id = self.odom_frame
        self.quatOdom.child_frame_id = self.child_frame
        self.quatOdom.pose.pose.position.x = self.odomNew.pose.pose.position.x
        self.quatOdom.pose.pose.position.y = self.odomNew.pose.pose.position.y
        self.quatOdom.pose.pose.position.z = self.odomNew.pose.pose.position.z
        self.quatOdom.pose.pose.orientation.x = q[1]
        self.quatOdom.pose.pose.orientation.y = q[2]
        self.quatOdom.pose.pose.orientation.z = q[3]
        self.quatOdom.pose.pose.orientation.w = q[0]
        self.quatOdom.twist.twist.linear.x = self.odomNew.twist.twist.linear.x
        self.quatOdom.twist.twist.linear.y = self.odomNew.twist.twist.linear.y
        self.quatOdom.twist.twist.linear.z = self.odomNew.twist.twist.linear.z
        self.quatOdom.twist.twist.angular.x = self.odomNew.twist.twist.angular.x
        self.quatOdom.twist.twist.angular.y = self.odomNew.twist.twist.angular.y
        self.quatOdom.twist.twist.angular.z = self.odomNew.twist.twist.angular.z

        for i in range(36):
            if i == 0 or i == 7 or i == 14:
                self.quatOdom.pose.covariance[i] = 0.01
            elif i == 21 or i == 28 or i == 35:
                self.quatOdom.pose.covariance[i] += 0.1
            else:
                self.quatOdom.pose.covariance[i] = 0

        self.odom_data_pub_quat.publish(self.quatOdom)

    # Update odometry information
    def update_odom(self):
        # Calculate the average distance
        cycleDistance = (self.distanceRight + self.distanceLeft) / 2.0

        # Calculate the number of radians the robot has turned since the last cycle
        turn_angle = (self.distanceRight - self.distanceLeft) / WHEEL_BASE_METERS
        cycleAngle = asin(max(min(1.0, turn_angle), -1.0))  # Constrain

        # Average angle during the last cycle
        avgAngle = cycleAngle / 2 + self.odomOld.pose.pose.orientation.z

        if avgAngle > pi:
            avgAngle -= 2 * pi
        elif avgAngle < -pi:
            avgAngle += 2 * pi

        # Calculate the new pose (x, y, and theta)
        self.odomNew.pose.pose.position.x = self.odomOld.pose.pose.position.x + cos(avgAngle) * cycleDistance
        self.odomNew.pose.pose.position.y = self.odomOld.pose.pose.position.y + sin(avgAngle) * cycleDistance
        self.odomNew.pose.pose.orientation.z = cycleAngle + self.odomOld.pose.pose.orientation.z

        # Prevent lockup from a single bad cycle
        if (
            isnan(self.odomNew.pose.pose.position.x)
            or isnan(self.odomNew.pose.pose.position.y)
            or isnan(self.odomNew.pose.pose.position.z)
        ):
            self.odomNew.pose.pose.position.x = self.odomOld.pose.pose.position.x
            self.odomNew.pose.pose.position.y = self.odomOld.pose.pose.position.y
            self.odomNew.pose.pose.orientation.z = self.odomOld.pose.pose.orientation.z

        # Make sure theta stays in the correct range
        if self.odomNew.pose.pose.orientation.z > pi:
            self.odomNew.pose.pose.orientation.z -= 2 * pi
        elif self.odomNew.pose.pose.orientation.z < -pi:
            self.odomNew.pose.pose.orientation.z += 2 * pi

        # Compute the velocity
        # self.get_clock().now().seconds_nanoseconds()
        self.odomNew.header.stamp = self.get_clock().now().to_msg()
        delta_t = (self.odomNew.header.stamp.nanosec - self.odomOld.header.stamp.nanosec) / 1000000000.0  # Duration in seconds
        if delta_t > 0:
            self.odomNew.twist.twist.linear.x = cycleDistance / delta_t
            self.odomNew.twist.twist.angular.z = cycleAngle / delta_t

        # Save the pose data for the next cycle
        self.odomOld.pose.pose.position.x = self.odomNew.pose.pose.position.x
        self.odomOld.pose.pose.position.y = self.odomNew.pose.pose.position.y
        self.odomOld.pose.pose.orientation.z = self.odomNew.pose.pose.orientation.z
        self.odomOld.header.stamp = self.odomNew.header.stamp

        # Publish the odometry message
        self.odom_data_pub_euler.publish(self.odomNew)


def main(args=None):
    rclpy.init(args=args)
    try:
        wheels_odom_node = WheelsOdom()
        rclpy.spin(wheels_odom_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        wheels_odom_node.destroy_node()
        wheels_odom_node.get_logger().info("wheel_odom_node node has shutdown")
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
