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
import colorsys
from random import random


class LedModes(object):
    SINGLE_COLOR = 0
    RAINBOW = 1
    STRIPE = 2
    COLOR_TURN = 3
    RANDOM = 4
    SINGLE_COLOR_CHANGE = 5


class NeopixelNode(Node):

    WHITE = (255, 255, 255, 255)
    RED = (255, 0, 0, 0)
    GREEN = (0, 255, 0, 0)
    BLUE = (0, 0, 255, 0)
    OFF = (0, 0, 0, 0)

    DEFAULT_COLOR = RED
    EXTRA_COLOR = BLUE
    DEFAULT_BRIGHTNESS = 0.6
    DEFAULT_ENABLED = True
    DEFAULT_MODE = LedModes.STRIPE
    NUM_PIXELS = 231   # 254 original

    #LED_ONE 50
    LED_ONE_START = 0
    LED_ONE_END = 49
    #LED_TWO 70
    LED_TWO_START = 50
    LED_TWO_END = 119
    #LED_THREE 14
    LED_THREE_START = 120
    LED_THREE_END = 133
    #LED_FOUR 13
    LED_FOUR_START = 134
    LED_FOUR_END = 146
    #LED_FIVE 34
    LED_FIVE_START = 147
    LED_FIVE_END = 179
    #LED_SIX 33
    LED_SIX_START = 180
    LED_SIX_END = 211
    #LED_SEVEN 16
    LED_SEVEN_START = 212
    LED_SEVEN_END = 227



    def __init__(self, com_pin=board.D21, num_pixels=NUM_PIXELS, pixel_order=neopixel.GRBW):
        super().__init__('neopixel_node')

        self.pixels = neopixel.NeoPixel(com_pin, num_pixels, brightness=self.DEFAULT_BRIGHTNESS, pixel_order=pixel_order, auto_write=False)
        self._pixel_order = pixel_order
        self._enabled = self.DEFAULT_ENABLED
        self._mode = self.DEFAULT_MODE
        self._color = self.DEFAULT_COLOR
        self._color2 = self.EXTRA_COLOR

        self._enable_sub = self.create_subscription(
            Bool, 'neopixel/enable', self._enable_callback, 1)
        self._color_sub = self.create_subscription(
            String, 'neopixel/color', self._color_callback, 1
        )
        self._brightness_sub = self.create_subscription(Float32, 'neopixel/brightness', self._brightness_callback, 1)
        self._mode_sub = self.create_subscription(UInt8, 'neopixel/mode', self._mode_callback, 1)

        self._update_rainbow_counter = 0
        self._timer = self.create_timer(0.001, self._update_light)

    def rainbow_cycle(self, update_function):
        j = self._update_rainbow_counter
        for i in range(self.NUM_PIXELS):
            pixel_index = (i * 256 // self.NUM_PIXELS) + j
            self.pixels[i] = update_function(pixel_index & 255)
        self.pixels.show()
        self._update_rainbow_counter += 1
        if self._update_rainbow_counter == 255:
            self._update_rainbow_counter = 0

    def wheel(self, pos):
        # Input a value 0 to 255 to get a color value.
        # The colours are a transition r - g - b - back to r.
        if pos < 0 or pos > 255:
            r = g = b = w = 0
        elif pos < 85:
            r = int(pos * 3)
            g = int(255 - pos * 3)
            b = 0
        elif pos < 170:
            pos -= 85
            r = int(255 - pos * 3)
            g = 0
            b = int(pos * 3)
        else:
            pos -= 170
            r = 0
            g = int(pos * 3)
            b = int(255 - pos * 3)
        w = 0
        return (r, g, b) if self._pixel_order == neopixel.RGB or self._pixel_order == neopixel.GRB else (r, g, b, w)

    def stripe(self, pos):
        if pos < 0 or pos > 231:
            r = g = b = w = 0
        else:

            r = g = b = w = 0
            if pos < 10:
                r, g, b, w = self._color

        return (r, g, b) if self._pixel_order == neopixel.RGB or self._pixel_order == neopixel.GRB else (r, g, b, w)

    def color_turn(self, pos):
        if pos < 0 or pos > 255:
            r = g = b = w = 0
        else:
            r, g, b, w = self._color
            r2, g2, b2, w2 = self._color2

            h, s, v = colorsys.rgb_to_hsv(r2/255, g2/255, b2/255)
            x = v / 18
            if pos > 245:
                v = v - (10 - (255-pos))*x
            elif pos < 10:
                v = v - (10 - pos)*x
            r2, g2, b2 = colorsys.hsv_to_rgb(h, s, v)

            r2 = int(r2*255)
            g2 = int(g2 * 255)
            b2 = int(b2 * 255)
            if pos > 245:
                r = r2
                g = g2
                b = b2
            elif pos < 10:
                r = r2
                g = g2
                b = b2
        w = 0
        return (r, g, b) if self._pixel_order == neopixel.RGB or self._pixel_order == neopixel.GRB else (r, g, b, w)

    def single_color_change(self, pos):
        if pos < 0 or pos > 255:
            r = g = b = w = 0
        else:
            range_ = 0.05
            r, g, b, w = self._color

            h, s, v = colorsys.rgb_to_hsv(r/255, g/255, b/255)

            if pos < 127:
                offset = ((pos/126) - 0.5) * range_
                h = (h + offset) % 1.0
            else:
                pos = 255 - pos
                offset = ((pos / 127) - 0.5) * range_
                h = (h + offset) % 1.0

            r, g, b = colorsys.hsv_to_rgb(h, s, v)
            r = int(r * 255)
            g = int(g * 255)
            b = int(b * 255)

        return (r, g, b) if self._pixel_order == neopixel.RGB or self._pixel_order == neopixel.GRB else (r, g, b, w)

    def random(self, pos):
        if pos < 0 or pos > 255:
            r = g = b = w = 0
        else:
            w = 0
            r, g, b = colorsys.hsv_to_rgb(random(), 1, random())

            r = int(r * 255)
            g = int(g * 255)
            b = int(b * 255)

        return (r, g, b) if self._pixel_order == neopixel.RGB or self._pixel_order == neopixel.GRB else (r, g, b, w)



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
        if self._mode == LedModes.RAINBOW:
            self.rainbow_cycle(self.wheel)
        elif self._mode == LedModes.STRIPE:
            self.rainbow_cycle(self.stripe)
        elif self._mode == LedModes.COLOR_TURN:
            self.rainbow_cycle(self.color_turn)
        elif self._mode == LedModes.RANDOM:
            self.rainbow_cycle(self.random())
        elif self._mode == LedModes.SINGLE_COLOR_CHANGE:
            self.rainbow_cycle(self.single_color_change)
        elif self._mode == LedModes.SINGLE_COLOR:
            if self._enabled:
                self.pixels.fill(self._color)
            else:
                self.pixels.fill(self.OFF)
            self.pixels.show()

    def _enable_callback(self, msg):
        self.get_logger().debug('Enable received {}'.format(msg.data))
        self._enabled = msg.data
        #self._update_light()

    def _color_callback(self, msg):
        self.get_logger().debug('Changing color {}'.format(msg.data))
        self._color = self._color_string_to_rgbw(msg.data)
        #self._update_light()

    def _mode_callback(self, msg):
        self.get_logger().debug('Changing mode {}'.format(msg.data))
        self._mode = msg.data
        #self._update_light()

    def _brightness_callback(self, msg):
        self.get_logger().debug('Changing brightness {}'.format(msg.data))
        self.pixels.brightness = msg.data
        #self._update_light()


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
