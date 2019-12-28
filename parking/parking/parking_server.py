#!/usr/bin/env python3

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import AddParkingSpot
from robot_interfaces.srv import DeleteParkingSpot
from visualization_msgs.msg import InteractiveMarker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


class ParkingSpotServer(Node):
    def __init__(self):
        super(ParkingSpotServer, self).__init__('parking_spot_server')
        self.spots = {}
        # self.marker_server = InteractiveMarkerServer(self, 'parking')
        # box_marker = InteractiveMarker(
        #     name='box',
        #     pose=Pose(
        #         position=Point(x=2, y=1, z=0),
        #         orientation=Quaternion(x=0, y=0, z=0, w=1),
        #     )
        # )
        # self.marker_server.insert(box_marker)

        self.add_srv = self.create_service(
            AddParkingSpot, 'add_parking_spot', self.add_spot)
        self.del_srv = self.create_service(
            DeleteParkingSpot, 'delete_parking_spot', self.delete_spot)

    def add_spot(self, request, response):
        self.spots[request.name] = request.pose
        response.success = True
        return response

    def delete_spot(self, request, response):
        del self.spots[request.name]
        response.success = True
        return response


def main():
    rclpy.init()
    node = ParkingSpotServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
