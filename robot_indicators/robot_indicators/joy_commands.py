import rclpy
from rclpy.node import Node
from robot_interfaces.srv import SetMuxSource
from sensor_msgs.msg import Joy


class JoyCommands(Node):
    def __init__(self):
        super(JoyCommands, self).__init__('joy_commands')
        self._subscription = self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        self._client = self.create_client(SetMuxSource, 'cmd_vel_source')
        self._buttons = {
            2: 'nav',
            3: 'joy',
        }
        while not self._client.wait_for_service(timeout_sec=1.0):
            print('cmd_vel_source service not available, waiting...')

    def joy_cb(self, msg):
        for btn, src in self._buttons.items():
            if msg.buttons[btn]:
                req = SetMuxSource.Request()
                req.name = src
                self._client.call_async(req)


def main():
    rclpy.init()
    node = JoyCommands()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
