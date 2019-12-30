#!/usr/bin/env python3
import math

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import AddParkingSpot
from robot_interfaces.srv import DeleteParkingSpot
from robot_interfaces.srv import GetParkingSpot
from robot_interfaces.srv import RenameParkingSpot
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker


# TODO(e) definitely have to make/find a linalg/utils library
def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.x = cy * cp * cr + sy * sp * sr
    q.y = cy * cp * sr - sy * sp * cr
    q.z = sy * cp * sr + cy * sp * cr
    q.w = sy * cp * cr - cy * sp * sr

    return q


class ParkingSpotServer(Node):
    def __init__(self):
        super(ParkingSpotServer, self).__init__('parking_spot_server')
        self.spots = {}
        self.marker_server = InteractiveMarkerServer(self, 'parking')
        # self.add_marker('test', Pose2D(x=1.1, y=0.5, theta=1.0))

        self.add_srv = self.create_service(
            AddParkingSpot, 'add_parking_spot', self.add_spot)
        self.del_srv = self.create_service(
            DeleteParkingSpot, 'delete_parking_spot', self.delete_spot)
        self.get_srv = self.create_service(
            GetParkingSpot, 'get_parking_spot', self.get_spot)
        self.rename_srv = self.create_service(
            RenameParkingSpot, 'rename_parking_spot', self.rename_spot)

    def box_feedback(self, fb) -> None:
        print(fb)

    def add_marker(self, name: str, pose: Pose2D) -> InteractiveMarker:
        marker = InteractiveMarker(
            name=name,
            pose=Pose(
                position=Point(x=pose.x, y=pose.y, z=0.),
                orientation=quaternion_from_euler(pose.theta, 0., 0.),
            )
        )
        marker.header.frame_id = 'map'

        sc = 0.2
        z = 0.1
        name_marker = Marker(
            type=Marker.TEXT_VIEW_FACING,
            scale=Vector3(x=sc, y=sc, z=sc),
            color=ColorRGBA(r=1., g=1., b=1., a=1.),
            text=name,
        )
        name_marker.pose.position.x = sc * -0.1
        name_marker.pose.position.z = z + sc * -0.1

        marker.controls = [
            InteractiveMarkerControl(
                name='name',
                orientation_mode=InteractiveMarkerControl.VIEW_FACING,
                interaction_mode=InteractiveMarkerControl.NONE,
                independent_marker_orientation=False,
                always_visible=True,
                markers=[name_marker],
            ),
            InteractiveMarkerControl(
                name='xaxis',
                orientation_mode=InteractiveMarkerControl.FIXED,
                orientation=Quaternion(x=0., y=0., z=0.7068252, w=0.7068252),
                interaction_mode=InteractiveMarkerControl.MOVE_AXIS,
            ),
            InteractiveMarkerControl(
                name='yaxis',
                orientation_mode=InteractiveMarkerControl.FIXED,
                interaction_mode=InteractiveMarkerControl.MOVE_AXIS,
            ),
            InteractiveMarkerControl(
                name='turn',
                interaction_mode=InteractiveMarkerControl.ROTATE_AXIS,
                orientation=Quaternion(x=0., y=0.7068252, z=0., w=0.7068252),
            )
        ]
        self.marker_server.insert(marker, feedback_callback=self.box_feedback)
        self.marker_server.applyChanges()
        return marker

    def add_spot(self, request, response):
        print(request)
        name = 'park{:02}'.format(len(self.spots) + 1)
        if name not in self.spots:
            marker = self.add_marker(name, request.pose)
            # self.spots[name] = marker
        self.spots[name] = request.pose
        response.success = True
        return response

    def delete_spot(self, request, response):
        try:
            del self.spots[request.name]
            response.success = True
        except KeyError:
            response.success = False
        return response

    def get_spot(self, request, response):
        try:
            response.pose = self.spots[request.name]
            response.success = True
        except KeyError:
            response.success = False
        return response

    def rename_spot(self, request, response):
        if request.name not in self.spots:
            response.success = False
            response.msg = 'Spot does not exist'
        elif request.new_name in self.spots:
            response.success = False
            response.msg = 'New name already taken'
        else:
            response.success = True
            spot = self.spots[request.name]
            del self.spots[request.name]
            self.spots[request.new_name] = spot
        return response


def main():
    rclpy.init()
    node = ParkingSpotServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
