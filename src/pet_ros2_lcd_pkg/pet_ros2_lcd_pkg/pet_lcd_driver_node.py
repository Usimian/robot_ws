"""
2 line LCD display node.

(c) https://github.com/Pet-Series
    https://github.com/Pet-Series/pet_ros2_lcd_pkg

Maintainer: stefan.kull@gmail.com
The MIT License (MIT)

Subscribe:
     /lcd_display/row1       # Text to display on top row
     /lcd_display/row2       # Text to display on bottom row

Prerequisite:
  $ sudo apt install i2c-tools
  $ sudo i2cdetect -y 1          <- Normaly 0x27 or 0x3F
  $ sudo chmod a+rw /dev/i2c-1   <- Give members of user/group i2c r/w to i2c interface.

Prerequisite:
  $ sudo pip3 install smbus2
  $ sudo pip3 install smbus

Launch sequence:
  1) $ ros2 run pet_ros2_lcd_pkg lcd_node
  2) $ ros2 topic pub /lcd_display/row3 std_msgs/msg/String 'data: Text on row 3' -1
"""

#  Include the ROS2 stuff...
from rcl_interfaces.msg import ParameterDescriptor

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import util.i2c_lib
import util.lcddriver

LCD_DISPLAY_WIDTH = 16

# TODO: Add the following 'behaviours to class 'LcdDisplayNode'
# lcd.lcd_clear()
# lcd.lcd_backlight('ON')
# lcd.lcd_backlight('OFF)


class LcdDisplayNode(Node):
    """Node to handle 2 line lcd display on I2C bus 1."""

    def __init__(self):
        """Init class."""
        super().__init__('pet_lcd_driver_node')

        # Set default Display-I2C address. Accessed via ROS Parameters...
        self.declare_parameter(
            'lcd_i2c_address', '0x27', ParameterDescriptor(description='LCD-display I2C address [default 0x27]')
            )
        self.LCD_I2C_ADDRESS_STR = self.get_parameter('lcd_i2c_address').get_parameter_value().string_value
        self.LCD_I2C_ADDRESS = int(self.LCD_I2C_ADDRESS_STR, 16)  # Convert Hex-string to Int

        # If topics ('lcd_display/row#') content update -> Then ...callback
        self.subscription_row1 = self.create_subscription(
            String, 'lcd_display/row1', self.lcd_update_row1_callback, 10
        )
        self.subscription_row2 = self.create_subscription(
            String, 'lcd_display/row2', self.lcd_update_row2_callback, 10
        )

        # Check we can open/contact the I2C-controller on the Display.
        try:
            # Initiate the LCD Display via I2C interface (include ./util/*.py)
            self.lcd = util.lcddriver.lcd(self.LCD_I2C_ADDRESS)

            # Put some info in the display
            self.lcd.display_string('lcd_driver_node', 1)
            self.lcd.display_string(f'I2C: {self.LCD_I2C_ADDRESS_STR}', 2)

            # Some basic information on the console
            self.get_logger().info('pet_lcd_driver_node STARTED')
            self.get_logger().info(f'I2C: <{str(hex(self.LCD_I2C_ADDRESS))}>')

        except Exception as ex:  # noqa
            # Note: a permission error can be fixed with a 'sudo chmod a+rw /dev/i2c-1'
            self.get_logger().error('pet_lcd_driver_node canceled:' + str(ex))
            # self.get_logger().error('pet_lcd_driver_node canceled:' + str(sys.exc_info()[1]))

    def lcd_update_row1_callback(self, msg):
        """Display msg on row 1 of lcd."""
        self.lcd.display_string(msg.data.ljust(LCD_DISPLAY_WIDTH, ' '), 1)
        # self.get_logger().info(f'LCD row 1 = '{msg.data}'')

    def lcd_update_row2_callback(self, msg):
        """Display msg on row 2 of lcd."""
        self.lcd.display_string(msg.data.ljust(LCD_DISPLAY_WIDTH, ' '), 2)
        # self.get_logger().info(f'LCD row 2 = '{msg.data}'')


def main(args=None):
    """
    ROS2 entrypoint.

    See also setup.py
    """
    rclpy.init(args=args)
    try:
        pet_lcd_driver_node = LcdDisplayNode()
        rclpy.spin(pet_lcd_driver_node)

    except KeyboardInterrupt:
        pass

    finally:
        pet_lcd_driver_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
