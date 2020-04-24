# Copyright 2020 Emerson Knapp
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from robot_interfaces.srv import SetMuxSource


class CmdVelMux(Node):
    choices = ['joy', 'nav']

    def __init__(self):
        self._subscriptions = [
            self.create_subscription(Twist, '/joy/cmd_vel', self.joy_cb, 10),
            self.create_subscription(Twist, '/nav/cmd_vel', self.nav_cb, 10),
        ]
        self._source = 'joy'
        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._set_srv = self.create_service(SetMuxSource, 'set_source', self.set_source)

    def joy_cb(self, msg):
        if self._source == 'joy':
            self._publisher.publish(msg)

    def nav_cb(self, msg):
        if self._source == 'nav':
            self._publisher.publish(msg)

    def set_source(self, request, response):
        if request.name not in self.choices:
            response.success = False
        else:
            self._source = request.name
            response.success = True
        return response


def main():
    rclpy.init()
    node = CmdVelMux()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
