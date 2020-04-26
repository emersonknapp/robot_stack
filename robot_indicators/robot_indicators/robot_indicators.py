#!/usr/bin/env python3
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class XPadLED(Node):
    ALL_FLASH = 1
    # blink 4 times then solid
    P1_CONNECT = 2
    P2_CONNECT = 3
    P3_CONNECT = 4
    P4_CONNECT = 5
    # set the player light to on without any flashing
    P1_ON = 6
    P2_ON = 7
    P3_ON = 8
    P4_ON = 9
    # light in circles!
    CYCLE = 10
    # flash current player 4 times then solid again
    PLAYER_RECONNECT = 11
    # super slow blink on current state (forever, probably)
    PLAYER_VERY_SLOW_BLINK = 12
    # alternating diagonal flashes for a while then back to previous state
    DIAG_FLASH = 13
    # all flash for a while then back to previous state
    ALL_SLOW_FLASH = 14
    # all light up then back to previous state
    ALL_LIGHT = 15

    def __init__(self):
        super(XPadLED, self).__init__('xpad_led')
        self._command = self.P2_CONNECT
        self._device = Path('/dev') / 'xled0'
        self._write_period = 1.0
        self._setting = self.P2_CONNECT
        self._timer = self.create_timer(self._write_period, self.write_light)
        self._sub = self.create_subscription(String, '/cmd_vel_mux_source', self._src_cb, 10)

    def write_light(self, off=False):
        if off:
            self._command = self.P1_CONNECT
        cmd_str = str(self._command)
        try:
            self._device.write_text(cmd_str)
        except FileNotFoundError:
            pass

    def _src_cb(self, msg):
        if msg.data == 'nav':
            self._command = self.P3_CONNECT
        elif msg.data == 'joy':
            self._command = self.P2_CONNECT
        else:
            self._command = self.DIAG_FLASH


def main():
    rclpy.init()
    node = XPadLED()
    rclpy.spin(node)
    rclpy.shutdown()
    node.write_light(False)


if __name__ == '__main__':
    main()
