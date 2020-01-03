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
            # cancel_callback=self.cancel_park_goal,
        )
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'NavigateToPose',
            callback_group=ReentrantCallbackGroup(),
        )
        self._get_parking_spot = self.create_client(GetParkingSpot, 'get_parking_spot')

    # def destroy(self):
    #     self._action_server.destroy()
    #     super().destroy_node()

    def receive_park_goal(self, goal_request):
        self.get_logger().info('RECEIVE PARK GOALLLL')
        return GoalResponse.ACCEPT

    # def cancel_park_goal(self, park_goal_handle):
    #     self.get_logger().info('Received park goal cancel request')
    #     future = self._nav_goal_handle.cancel_goal_async()
    #     future.add_done_callback(self.cancel_done)
    #     return CancelResponse.ACCEPT

    async def execute_park_goal(self, park_goal_handle):
        self.get_logger().info('Executing park goal...')
        request = park_goal_handle.request
        # rclpy.spin_until_future_complete(self, self._send_goal_future)
        # rclpy.spin_until_future_complete(self, self._get_result_future)
        self.get_logger().info('Received park goal request {} {}'.format(type(request), request))
        self.get_logger().info('Finding parking spot {}'.format(request.name))

        # Retrieve parking spot Pose 2D
        response = self._get_parking_spot.call(
            GetParkingSpot.Request(name=request.name))
        if not response.success:
            self.get_logger().info('Couldnt get that parking spot')
            return GoalResponse.REJECT
        self.get_logger().info('Found parking spot')

        # Convert Pose2D into 3D pose for nav
        spot2d = response.pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = spot2d.x
        pose_msg.pose.position.y = spot2d.y
        q = quaternion_from_euler(spot2d.theta, 0., 0.)
        x, y, z, w = q
        pose_msg.pose.orientation = Quaternion(x=x, y=y, z=z, w=w)

        nav_goal_msg = NavigateToPose.Goal()
        nav_goal_msg.pose = pose_msg

        # Do the action portion
        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(nav_goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Nav goal rejected :(')
            result = NavigateToParkingSpot.Result()
            return result

        self.get_logger().info('Nav Goal accepted :)')

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        status = get_result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav goal succeeded, succeeding park goal!')
            park_goal_handle.succeed()
        else:
            self.get_logger().info('Nav goal failed with status: {0}'.format(status))
            park_goal_handle.fail()

        result = NavigateToParkingSpot.Result()
        return result

    # def nav_goal_response(self, future):
    #     nav_goal_handle = future.result()
    #
    #     if not nav_goal_handle.accepted:
    #         self.get_logger().info('Nav goal rejected :(')
    #         return
    #
    #     self.get_logger().info('Nav Goal accepted :)')
    #     self._get_result_future = nav_goal_handle.get_result_async()

    # def nav_goal_feedback(self, feedback_msg):
    #     feedback = feedback_msg.feedback
    #     self.get_logger().info('Received nav goal feedback: {}'.format(feedback))


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = NavigateToParkingSpotActionServer(executor)
    executor.add_node(node)
    executor.spin()
    # node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
