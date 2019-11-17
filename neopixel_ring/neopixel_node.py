#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import Header, Bool
import board
import neopixel


class NeopixelNode(Node):

    WHITE = (255, 255, 255)
    RED = (255, 0, 0)
    BLUE = (0, 0, 255)
    OFF = (0, 0, 0)

    DEFAULT_BRIGHTNESS = 1.0

    def __init__(self, com_pin=board.D18, num_pixels=16):
        super().__init__('neopixel_node')

        self.pixels = neopixel.NeoPixel(com_pin, num_pixels, brightness=self.DEFAULT_BRIGHTNESS)
        self._connected = False
        self._enabled = False
        self._update_light()

        self.declare_parameter('heartbeat_period_s', 0.5)
        self._timeout_s = self.get_parameter('heartbeat_period_s').value * 2.0

        secs = int(self._timeout_s)
        nsecs = int((self._timeout_s - secs) * 1e9)
        self._timeout_duration = Duration(seconds=secs, nanoseconds=nsecs)

        self.get_logger().info('Starting with timeout {}'.format(self._timeout_s))
        self._clock = Clock()
        self._enable_sub = self.create_subscription(
            Bool, 'neopixel/enable', self._enable_callback, 1)
        self._heartbeat_sub = self.create_subscription(
            Header, 'heartbeat', self._heartbeat_callback, 1)
        self._timer = self.create_timer(self._timeout_s, self._timer_callback)
        self._timer.cancel()

    def _standby_light(self):
        self.pixels.fill(self.OFF)
        color = tuple(map(lambda x: int(x * 0.01),
                          (self.BLUE if self._connected else self.RED)))
        for i in range(0, len(self.pixels), 2):
            self.pixels[i] = color

    def _update_light(self):
        if self._enabled and self._connected:
            self.pixels.fill(self.RED)
        else:
            self._standby_light()

    def _set_connected(self, value):
        if value is self._connected:
            return

        self.get_logger().debug('connected: {}'.format(value))
        self._connected = value
        self._update_light()

    def _enable_callback(self, msg):
        self.get_logger().debug('Enable received {}'.format(msg.data))
        self._enabled = msg.data
        self._update_light()

    def _heartbeat_callback(self, msg):
        self.get_logger().debug('Heartbeat received {}'.format(msg.stamp))

        current = Time.from_msg(self._clock.now().to_msg())
        if (current - Time.from_msg(msg.stamp)) > self._timeout_duration:
            return

        self._set_connected(True)
        self._timer.reset()

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
