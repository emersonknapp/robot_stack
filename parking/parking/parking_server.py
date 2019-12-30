#!/usr/bin/env python3

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import AddParkingSpot
from robot_interfaces.srv import DeleteParkingSpot
from robot_interfaces.srv import GetParkingSpot
from robot_interfaces.srv import RenameParkingSpot
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import MenuEntry


class ParkingSpotServer(Node):
    def __init__(self):
        super(ParkingSpotServer, self).__init__('parking_spot_server')
        self.spots = {}
        self.marker_server = InteractiveMarkerServer(self, 'parking')
        box_marker = InteractiveMarker(
            name='box',
            pose=Pose(
                position=Point(x=2., y=1., z=0.),
                orientation=Quaternion(x=0., y=0., z=0., w=1.),
            )
        )
        box_marker.header.frame_id = 'map'
        box_marker.menu_entries.append(
            MenuEntry(id=1, parent_id=0, title='florp', command_type=0)
        )
        box_marker.controls = [
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
        self.marker_server.insert(box_marker, feedback_callback=self.box_feedback)
        self.marker_server.applyChanges()

        self.add_srv = self.create_service(
            AddParkingSpot, 'add_parking_spot', self.add_spot)
        self.del_srv = self.create_service(
            DeleteParkingSpot, 'delete_parking_spot', self.delete_spot)
        self.get_srv = self.create_service(
            GetParkingSpot, 'get_parking_spot', self.get_spot)
        self.rename_srv = self.create_service(
            RenameParkingSpot, 'rename_parking_spot', self.rename_spot)

    def box_feedback(self, fb):
        print(fb)

    def add_spot(self, request, response):
        print(request)
        name = 'park{:02}'.format(len(self.spots))
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
