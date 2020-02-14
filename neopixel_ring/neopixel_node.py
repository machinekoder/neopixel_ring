#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import Bool, String, UInt8, Float32, Header
import board
import neopixel
import colorsys
from random import random


COM_PIN = board.D18
PIXEL_ORDER = neopixel.GRBW
NUM_BODY_PIXELS = 227  # 254 original
NUM_HEADLIGHT_PIXELS = 12
NUM_PIXELS = NUM_BODY_PIXELS + NUM_HEADLIGHT_PIXELS
STANDBY_BRIGHTNESS = 0.1

# LED_ONE 50
LED_ONE_START = 0
LED_ONE_END = 49
# LED_TWO 70
LED_TWO_START = 50
LED_TWO_END = 119
# LED_THREE 14
LED_THREE_START = 120
LED_THREE_END = 133
# LED_FOUR 13
LED_FOUR_START = 134
LED_FOUR_END = 146
# LED_FIVE 34
LED_FIVE_START = 147
LED_FIVE_END = 179
# LED_SIX 33
LED_SIX_START = 180
LED_SIX_END = 211
# LED_SEVEN 16
LED_SEVEN_START = 212
LED_SEVEN_END = 227


class LedModes(object):
    SINGLE_COLOR = 0
    RAINBOW = 1
    STRIPE = 2
    COLOR_TURN = 3
    RANDOM = 4
    SINGLE_COLOR_CHANGE = 5


class LedColors(object):
    WHITE = (255, 255, 255, 255)
    RED = (255, 0, 0, 0)
    REDW = (255, 0, 0, 128)
    GREEN = (0, 255, 0, 0)
    BLUE = (0, 0, 255, 0)
    OFF = (0, 0, 0, 0)


class LedSegment(object):
    def __init__(
        self,
        node,
        name,
        pixels,
        start_index,
        num_pixels,
        pixel_order,
        enabled,
        mode,
        color,
        extra_color,
        brightness
    ):
        self._node = node
        self._name = name
        self._start_index = start_index
        self._num_pixels = num_pixels
        self._pixels = pixels

        self._pixel_order = pixel_order
        self._enabled = enabled
        self._mode = mode
        self._color = color
        self._color2 = extra_color
        self._brightness = brightness
        self.connected = False

        self._subs = [
            node.create_subscription(
                Bool, 'neopixel/{}/enable'.format(name), self._enable_callback, 1
            ),
            node.create_subscription(
                String, 'neopixel/{}/color'.format(name), self._color_callback, 1
            ),
            node.create_subscription(
                String, 'neopixel/{}/extra_color'.format(name), self._extra_color_callback, 1
            ),
            node.create_subscription(
                UInt8, 'neopixel/{}/mode'.format(name), self._mode_callback, 1
            ),
            node.create_subscription(
                Float32, 'neopixel/{}/brightness'.format(name), self._brightness_callback, 1
            )
        ]

        self._update_rainbow_counter = 0

    def update_light(self):
        if self.connected:
            self._pixels.brightness = self._brightness
            if self._enabled:
                if self._mode == LedModes.RAINBOW:
                    self._rainbow_cycle(self._wheel)
                elif self._mode == LedModes.STRIPE:
                    self._rainbow_cycle(self._stripe)
                elif self._mode == LedModes.COLOR_TURN:
                    self._rainbow_cycle(self._color_turn)
                elif self._mode == LedModes.RANDOM:
                    self._rainbow_cycle(self._random)
                elif self._mode == LedModes.SINGLE_COLOR_CHANGE:
                    self._rainbow_cycle(self._single_color_change)
                elif self._mode == LedModes.SINGLE_COLOR:
                    for i in range(self._start_index, self._start_index + self._num_pixels):
                        self._pixels[i] = self._color
                else:
                    self._node.get_logger().warn('Unknown mode {}'.format(self._mode))
            else:
                for i in range(self._start_index, self._start_index + self._num_pixels):
                    self._pixels[i] = LedColors.OFF
        else:
            self._pixels.brightness = STANDBY_BRIGHTNESS
            self._rainbow_cycle(self._standby_light)

    def _rainbow_cycle(self, update_function):
        j = self._update_rainbow_counter
        for i in range(self._num_pixels):
            pixel_index = (i * 256 // self._num_pixels) + j
            self._pixels[i + self._start_index] = update_function(pixel_index & 255)
        self._update_rainbow_counter += 1
        if self._update_rainbow_counter == 255:
            self._update_rainbow_counter = 0

    def _wheel(self, pos):
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
        return (
            (r, g, b)
            if self._pixel_order == neopixel.RGB or self._pixel_order == neopixel.GRB
            else (r, g, b, w)
        )

    def _stripe(self, pos):
        if pos < 0 or pos > 231:
            r = g = b = w = 0
        else:

            r = g = b = w = 0
            if pos < 10:
                r, g, b, w = self._color

        return (
            (r, g, b)
            if self._pixel_order == neopixel.RGB or self._pixel_order == neopixel.GRB
            else (r, g, b, w)
        )

    def _color_turn(self, pos):
        if pos < 0 or pos > 255:
            r = g = b = w = 0
        else:
            r, g, b, w = self._color
            r2, g2, b2, w2 = self._color2

            h, s, v = colorsys.rgb_to_hsv(r2 / 255, g2 / 255, b2 / 255)
            x = v / 18
            if pos > 245:
                v = v - (10 - (255 - pos)) * x
            elif pos < 10:
                v = v - (10 - pos) * x
            r2, g2, b2 = colorsys.hsv_to_rgb(h, s, v)

            r2 = int(r2 * 255)
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
        return (
            (r, g, b)
            if self._pixel_order == neopixel.RGB or self._pixel_order == neopixel.GRB
            else (r, g, b, w)
        )

    def _single_color_change(self, pos):
        if pos < 0 or pos > 255:
            r = g = b = w = 0
        else:
            range_ = 0.05
            r, g, b, w = self._color

            h, s, v = colorsys.rgb_to_hsv(r / 255, g / 255, b / 255)

            if pos < 127:
                offset = ((pos / 126) - 0.5) * range_
                h = (h + offset) % 1.0
            else:
                pos = 255 - pos
                offset = ((pos / 127) - 0.5) * range_
                h = (h + offset) % 1.0

            r, g, b = colorsys.hsv_to_rgb(h, s, v)
            r = int(r * 255)
            g = int(g * 255)
            b = int(b * 255)

        return (
            (r, g, b)
            if self._pixel_order == neopixel.RGB or self._pixel_order == neopixel.GRB
            else (r, g, b, w)
        )

    def _random(self, pos):
        if pos < 0 or pos > 255:
            r = g = b = w = 0
        else:
            w = 0
            r, g, b = colorsys.hsv_to_rgb(random(), 1, random())

            r = int(r * 255)
            g = int(g * 255)
            b = int(b * 255)

        return (
            (r, g, b)
            if self._pixel_order == neopixel.RGB or self._pixel_order == neopixel.GRB
            else (r, g, b, w)
        )

    def _standby_light(self, pos):
        if pos % 50 == 0:
            r, g, b, w = LedColors.RED
        else:
            r = g = b = w = 0

        return (
            (r, g, b)
            if self._pixel_order == neopixel.RGB or self._pixel_order == neopixel.GRB
            else (r, g, b, w)
        )

    @staticmethod
    def _color_string_to_rgbw(string):
        if not string.startswith('#') or len(string) != 9:
            return 0, 0, 0, 0
        try:
            string = string.lstrip('#')
            data = tuple(int(string[i : i + 2], 16) for i in (0, 2, 4, 6))
        except TypeError:
            return 0, 0, 0, 0
        else:
            return data

    def _enable_callback(self, msg):
        self._node.get_logger().debug('Enable received {}'.format(msg.data))
        self._enabled = msg.data

    def _color_callback(self, msg):
        self._node.get_logger().debug('Changing color {}'.format(msg.data))
        self._color = self._color_string_to_rgbw(msg.data)

    def _extra_color_callback(self, msg):
        self._node.get_logger().debug('Changing extra color {}'.format(msg.data))
        self._color2 = self._color_string_to_rgbw(msg.data)

    def _mode_callback(self, msg):
        self._node.get_logger().debug('Changing mode {}'.format(msg.data))
        self._mode = msg.data

    def _brightness_callback(self, msg):
        self._node.get_logger().debug('Changing brightness {}'.format(msg.data))
        self._brightness = msg.data


class NeopixelNode(Node):
    UPDATE_INTERVAL_S = 0.001

    def __init__(
        self, com_pin=COM_PIN, num_pixels=NUM_PIXELS, pixel_order=PIXEL_ORDER
    ):
        super().__init__('neopixel_node')

        self._pixels = neopixel.NeoPixel(
            com_pin,
            num_pixels,
            brightness=0.6,
            pixel_order=pixel_order,
            auto_write=False,
        )
        self._connected = False

        self.declare_parameter('heartbeat_period_s', 0.5)
        self._timeout_s = self.get_parameter('heartbeat_period_s').value * 2.0
        secs = int(self._timeout_s)
        nsecs = int((self._timeout_s - secs) * 1e9)
        self._timeout_duration = Duration(seconds=secs, nanoseconds=nsecs)

        self._segments = [
            LedSegment(
                node=self,
                pixels=self._pixels,
                name='body',
                start_index=0,
                num_pixels=NUM_BODY_PIXELS,
                pixel_order=pixel_order,
                enabled=True,
                mode=LedModes.STRIPE,
                color=LedColors.RED,
                extra_color=LedColors.BLUE,
                brightness=0.6
            ),
            LedSegment(
                node=self,
                pixels=self._pixels,
                name='headlight',
                start_index=NUM_BODY_PIXELS,
                num_pixels=NUM_HEADLIGHT_PIXELS,
                pixel_order=pixel_order,
                enabled=False,
                mode=LedModes.SINGLE_COLOR,
                color=LedColors.REDW,
                extra_color=LedColors.BLUE,
                brightness=1.0
            ),
        ]
        self._heartbeat_sub = self.create_subscription(
            Header, 'heartbeat', self._heartbeat_callback, 1)

        self._update_timer = self.create_timer(self.UPDATE_INTERVAL_S, self._update_light)
        self._timeout_timer = self.create_timer(self._timeout_s, self._timeout)
        self._timeout_timer.cancel()

    def _update_light(self):
        for segment in self._segments:
            segment.connected = self._connected
            segment.update_light()
        self._pixels.show()

    def _heartbeat_callback(self, msg):
        self.get_logger().debug('Heartbeat received {}'.format(msg.stamp))

        current = Time.from_msg(self._clock.now().to_msg())
        if (current - Time.from_msg(msg.stamp)) > self._timeout_duration:
            return
        if not self._connected:
            self.get_logger().debug('connected')
            self._connected = True
        self._update_timer.reset()

    def _timeout(self):
        self._timer.cancel()
        self.get_logger().debug('disconnected')
        self._connected = False


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
