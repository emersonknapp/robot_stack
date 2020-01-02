#!/usr/bin/env python3
from math import cos, sin
from pathlib import Path

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
import yaml


# TODO(e) definitely have to make/find a linalg/utils library
def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    return Quaternion(
        x=cy * cp * cr + sy * sp * sr,
        y=cy * cp * sr - sy * sp * cr,
        z=sy * cp * sr + cy * sp * cr,
        w=sy * cp * cr - cy * sp * sr)


class ParkingSpotServer(Node):
    def __init__(self):
        super(ParkingSpotServer, self).__init__('parking_spot_server')
        self.declare_parameter('map_yaml')

        self.markers = {}
        self.poses = {}
        self.marker_server = InteractiveMarkerServer(self, 'parking')
        map_param = self.get_parameter('map_yaml').value
        self.map_path = Path(map_param)
        self.load_map()

        self.add_srv = self.create_service(
            AddParkingSpot, 'add_parking_spot', self.add_spot)
        self.del_srv = self.create_service(
            DeleteParkingSpot, 'delete_parking_spot', self.delete_spot)
        self.get_srv = self.create_service(
            GetParkingSpot, 'get_parking_spot', self.get_spot)
        self.rename_srv = self.create_service(
            RenameParkingSpot, 'rename_parking_spot', self.rename_spot)

    def save_map(self):
        self.map_data['parking'] = {
            name: [pose.x, pose.y, pose.theta]
            for name, pose in self.poses.items()
        }
        with self.map_path.open('w') as map_yaml:
            yaml.dump(self.map_data, map_yaml)

    def load_map(self):
        with self.map_path.open('r') as map_yaml:
            self.map_data = yaml.safe_load(map_yaml)
        self.poses = {
            name: Pose2D(x=pose[0], y=pose[1], theta=pose[2])
            for name, pose in self.map_data.get('parking', {}).items()
        }
        self.markers = {
            name: self.add_marker(name, Pose2D(x=pose[0], y=pose[1], theta=pose[2]))
            for name, pose in self.map_data.get('parking', {}).items()
        }

    def box_feedback(self, fb) -> None:
        # TODO(e) save on finished move feedback
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
        name = 'park{:02}'.format(len(self.poses) + 1)
        if name in self.poses:
            response.success = False
            return response

        marker = self.add_marker(name, request.pose)
        self.poses[name] = request.pose
        self.markers[name] = marker
        response.success = True
        self.save_map()
        return response

    def delete_spot(self, request, response):
        try:
            del self.poses[request.name]
            self.marker_server.erase(request.name)
            self.marker_server.applyChanges()
            del self.markers[request.name]
            response.success = True
            self.save_map()
        except KeyError:
            response.success = False
        return response

    def get_spot(self, request, response):
        try:
            response.pose = self.poses[request.name]
            response.success = True
        except KeyError:
            response.success = False
        return response

    def rename_spot(self, request, response):
        if request.name not in self.poses:
            response.success = False
            response.msg = 'Spot does not exist'
        elif request.new_name in self.poses:
            response.success = False
            response.msg = 'New name already taken'
        else:
            response.success = True
            name = request.name
            pose = self.poses[name]
            del self.markers[name]
            del self.poses[name]
            self.poses[request.new_name] = pose
            self.marker_server.erase(name)
            marker = self.add_marker(request.new_name, pose)
            self.markers[request.new_name] = marker
            self.save_map()
        return response


def main():
    rclpy.init()
    node = ParkingSpotServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
