#!/usr/bin/env python3
from pathlib import Path

import rclpy


class XPadLED:
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
        self.device = Path('/dev') / 'xled0'
        self.write_period = 1.0

    def write_light(self, on: bool = True):
        if on:
            command = self.P2_CONNECT
        else:
            command = self.P1_CONNECT
        cmd_str = str(command)
        try:
            self.device.write_text(cmd_str)
        except FileNotFoundError:
            pass


def main():
    rclpy.init()
    node = rclpy.create_node('robot_indicators')
    led = XPadLED()
    timer = node.create_timer(led.write_period, lambda: led.write_light(True))
    rclpy.spin(node)
    timer.cancel()
    rclpy.shutdown()
    led.write_light(False)


if __name__ == '__main__':
    main()
