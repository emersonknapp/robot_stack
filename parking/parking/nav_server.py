# Copyright 2019 Emerson Knapp
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
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import (
    PoseStamped,
    Quaternion,
)
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_interfaces.action import NavigateToParkingSpot
from robot_interfaces.srv import GetParkingSpot

import threading

from .transformations import quaternion_from_euler


class NavigateToParkingSpotActionServer(Node):
    def __init__(self, executor):
        super().__init__('nav_to_parkspot')
        self.executor = executor
        self._action_server = ActionServer(
            self,
            NavigateToParkingSpot,
            'navigate_to_parking_spot',
            execute_callback=self.execute_park_goal,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.receive_park_goal,
            cancel_callback=self.cancel_park_goal,
        )
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'NavigateToPose',
        )
        self._get_parking_spot = self.create_client(GetParkingSpot, 'get_parking_spot')
        self.has_nav_goal_event = threading.Event()

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def receive_park_goal(self, goal_request):
        self.get_logger().info('Received park goal request {} {}'.format(
            type(goal_request), goal_request))
        nav_goal_msg = NavigateToPose.Goal()
        spot_name = goal_request.name
        self.get_logger().info('Finding parking spot {}'.format(spot_name))
        response = self._get_parking_spot.call(
            GetParkingSpot.Request(name=spot_name))
        if not response.success:
            self.get_logger().info('Couldnt get that parking spot')
            return GoalResponse.REJECT
        self.get_logger().info('Found parking spot')

        spot2d = response.pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = spot2d.x
        pose_msg.pose.position.y = spot2d.y
        q = quaternion_from_euler(spot2d.theta, 0., 0.)
        x, y, z, w = q
        pose_msg.pose.orientation = Quaternion(x=x, y=y, z=z, w=w)
        self.get_logger().info('Finished constructing pose {}'.format(pose_msg))
        nav_goal_msg.pose = pose_msg

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            nav_goal_msg,
            feedback_callback=self.nav_goal_feedback)
        self._send_goal_future.add_done_callback(self.nav_goal_response)
        return GoalResponse.ACCEPT

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

    def cancel_park_goal(self, park_goal_handle):
        self.get_logger().info('Received park goal cancel request')
        future = self._nav_goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)
        return CancelResponse.ACCEPT

    async def execute_park_goal(self, park_goal_handle):
        self.get_logger().info('Executing park goal...')

        rclpy.spin_until_future_complete(self, self._send_goal_future)
        self.has_nav_goal_event.wait()
        rclpy.spin_until_future_complete(self, self._get_result_future)

        status = self._get_result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav goal succeeded, succeeding park goal!')
            park_goal_handle.succeed()
        else:
            self.get_logger().info('Nav goal failed with status: {0}'.format(status))
            park_goal_handle.fail()

        result = NavigateToParkingSpot.Result()
        return result

    def nav_goal_response(self, future):
        nav_goal_handle = future.result()

        if not nav_goal_handle.accepted:
            self.get_logger().info('Nav goal rejected :(')
            return

        self.get_logger().info('Nav Goal accepted :)')
        self._get_result_future = nav_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav_goal_result)
        self.has_nav_goal_event.set()

    def nav_goal_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received nav goal feedback: {}'.format(feedback))

    def nav_goal_result(self, future):
        result = future.result().result
        self.get_logger().info('Nav Goal Result: {}'.format(result))


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = NavigateToParkingSpotActionServer(executor)
    executor.add_node(node)
    executor.spin()
    node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
