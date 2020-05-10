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
import prometheus_client

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState


class PrometheusExporter(Node):
    def __init__(self):
        super(PrometheusExporter, self).__init__('prometheus_exporter')
        self._sub = self.create_subscription(
            BatteryState, '/battery', self.battery_cb, qos_profile_sensor_data)
        self._gauge = prometheus_client.Gauge('battery_percent', 'Battery Percentage')

    def battery_cb(self, msg):
        self._gauge.set(msg.percentage)


def main():
    prometheus_client.start_http_server(9101)
    rclpy.init()
    node = PrometheusExporter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
