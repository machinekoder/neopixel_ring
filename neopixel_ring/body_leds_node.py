#!/usr/bin/env python3
import struct

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import Bool, String, UInt8, Float32
import board
import neopixel

class NeopixelNode(Node):

    WHITE = (255, 255, 255, 255)
    RED = (255, 0, 0, 0)
    GREEN = (0, 255, 0, 0)
    BLUE = (0, 0, 255, 0)
    OFF = (0, 0, 0, 0)

    DEFAULT_COLOR = GREEN
    DEFAULT_BRIGHTNESS = 0.2
    DEFAULT_ENABLED = False
    DEFAULT_MODE = 0

    def __init__(self, com_pin=board.D21, num_pixels=254, pixel_order=neopixel.GRBW):
        super().__init__('neopixel_node')

        self.pixels = neopixel.NeoPixel(com_pin, num_pixels, brightness=self.DEFAULT_BRIGHTNESS, pixel_order=pixel_order)
        self._enabled = self.DEFAULT_ENABLED
        self._mode = self.DEFAULT_MODE
        self._color = self.DEFAULT_COLOR
        self._update_light()

        self._enable_sub = self.create_subscription(
            Bool, 'neopixel/enable', self._enable_callback, 1)
        self._color_sub = self.create_subscription(
            String, 'neopixel/color', self._color_callback, 1
        )
        self._brightness_sub = self.create_subscription(Float32, 'neopixel/brightness', self._brightness_callback, 1)
        self._mode_sub = self.create_subscription(UInt8, 'neopixel/mode', self._mode_callback, 1)

    @staticmethod
    def _color_string_to_rgbw(string):
        if not string.startswith('#') or len(string) != 9:
            return (0, 0, 0, 0)
        try:
            string = string.lstrip('#')
            data = tuple(int(string[i:i+2], 16) for i in (0, 2, 4, 6))
        except TypeError:
            return (0, 0, 0, 0)
        else:
            return data

    def _update_light(self):
        if self._enabled:
            self.pixels.fill(self._color)
        else:
            self.pixels.fill(self.OFF)

    def _enable_callback(self, msg):
        self.get_logger().debug('Enable received {}'.format(msg.data))
        self._enabled = msg.data
        self._update_light()

    def _color_callback(self, msg):
        self.get_logger().debug('Changing color {}'.format(msg.data))
        self._color = self._color_string_to_rgbw(msg.data)
        self._update_light()

    def _mode_callback(self, msg):
        self.get_logger().debug('Changing mode {}'.format(msg.data))
        self._mode = msg.data
        self._update_light()

    def _brightness_callback(self, msg):
        self.get_logger().debug('Changing brightness {}'.format(msg.data))
        self.pixels.brightness = msg.data
        self._update_light()

    def _timer_callback(self):
        self._timer.cancel()
        self._set_connected(False)


def main(args=None):
    rclpy.init(args=args)

    node = NeopixelNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
