#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Interoperability Serialization Tests."""

import rospy
import rosunit
import numpy as np
from unittest import TestCase
from cv_bridge import CvBridge
from interop import serializers
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker
from interop.msg import Color, Orientation, Shape, Object, ObjectType


class TestSerializers(TestCase):

    """Tests interoperability serializers."""

    def test_mission_deserializer(self):
        """Tests the mission deserializer."""
        # yapf: disable
        data = {
            "id": 1,
            "active": True,
            "air_drop_pos": {
                "latitude": 38.141833,
                "longitude": -76.425263
            },
            "fly_zones": [{
                "altitude_msl_max":
                    200.0,
                "altitude_msl_min":
                    100.0,
                "boundary_pts": [{
                    "latitude": 38.142544,
                    "longitude": -76.434088,
                    "order": 1
                }, {
                    "latitude": 38.141833,
                    "longitude": -76.425263,
                    "order": 2
                }, {
                    "latitude": 38.144678,
                    "longitude": -76.427995,
                    "order": 3
                }]
            }],
            "home_pos": {
                "latitude": 38.14792,
                "longitude": -76.427995
            },
            "mission_waypoints": [{
                "altitude_msl": 200.0,
                "latitude": 38.142544,
                "longitude": -76.434088,
                "order": 1
            }],
            "off_axis_odlc_pos": {
                "latitude": 38.142544,
                "longitude": -76.434088
            },
            "emergent_last_known_pos": {
                "latitude": 38.145823,
                "longitude": -76.422396
            },
            "search_grid_points": [{
                "altitude_msl": 200.0,
                "latitude": 38.142544,
                "longitude": -76.434088,
                "order": 1
            }]
        }
        # yapf: enable

        mission = serializers.MissionDeserializer.from_dict(data, "map")
        flyzones = mission[0]
        search_grid = mission[1]
        waypoints = mission[2]
        air_drop_pos = mission[3]
        off_axis_obj = mission[4]
        emergent_obj = mission[5]
        home_pos = mission[6]

        # Test flyzones.
        self.assertEqual(len(data["fly_zones"]), len(flyzones.flyzones))
        for i, flyzone in enumerate(flyzones.flyzones):
            zone = data["fly_zones"][i]
            max_alt = serializers.feet_to_meters(zone["altitude_msl_max"])
            min_alt = serializers.feet_to_meters(zone["altitude_msl_min"])

            self.assertEqual(flyzone.max_alt, max_alt)
            self.assertEqual(flyzone.min_alt, min_alt)
            self.assertEqual(
                len(flyzone.zone.polygon.points), len(zone["boundary_pts"]))

            bound = zone["boundary_pts"]
            for k, pnt in enumerate(flyzone.zone.polygon.points):
                self.assertEqual(k + 1, bound[k]["order"])

                self.assertEqual(pnt.latitude, bound[k]["latitude"])
                self.assertEqual(pnt.longitude, bound[k]["longitude"])

        # Test search grid.
        grid = data["search_grid_points"]
        self.assertEqual(len(search_grid.polygon.points), len(grid))
        for i, pnt in enumerate(search_grid.polygon.points):
            self.assertEqual(i + 1, grid[i]["order"])

            altitude = serializers.feet_to_meters(grid[i]["altitude_msl"])

            self.assertEqual(pnt.latitude, grid[i]["latitude"])
            self.assertEqual(pnt.longitude, grid[i]["longitude"])
            self.assertEqual(pnt.altitude, altitude)

        # Test waypoints.
        points = data["mission_waypoints"]
        self.assertEqual(len(waypoints.waypoints), len(points))

        for i, pnt in enumerate(waypoints.waypoints):
            self.assertEqual(i + 1, points[i]["order"])

            altitude = serializers.feet_to_meters(points[i]["altitude_msl"])

            self.assertEqual(pnt.latitude, points[i]["latitude"])
            self.assertEqual(pnt.longitude, points[i]["longitude"])
            self.assertEqual(pnt.altitude, altitude)

        # Test airdrop pos.
        self.assertEqual(air_drop_pos.position.latitude,
                         data["air_drop_pos"]["latitude"])
        self.assertEqual(air_drop_pos.position.longitude,
                         data["air_drop_pos"]["longitude"])

        # Test off axis object.
        self.assertEqual(off_axis_obj.position.latitude,
                         data["off_axis_odlc_pos"]["latitude"])
        self.assertEqual(off_axis_obj.position.longitude,
                         data["off_axis_odlc_pos"]["longitude"])

        # Test emergent object.
        self.assertEqual(emergent_obj.position.latitude,
                         data["emergent_last_known_pos"]["latitude"])
        self.assertEqual(emergent_obj.position.longitude,
                         data["emergent_last_known_pos"]["longitude"])

        # Test home position.
        self.assertEqual(home_pos.position.latitude,
                         data["home_pos"]["latitude"])
        self.assertEqual(home_pos.position.longitude,
                         data["home_pos"]["longitude"])

    def test_obstacles_deserializer(self):
        """Tests obstacles deserializer."""
        # Set up test data.
        data = {
            "moving_obstacles": [{
                "altitude_msl": 189.56748784643966,
                "latitude": 38.141826869853645,
                "longitude": -76.43199876559223,
                "sphere_radius": 150.0
            }, {
                "altitude_msl": 250.0,
                "latitude": 38.14923628783763,
                "longitude": -76.43238529543882,
                "sphere_radius": 150.0
            }],
            "stationary_obstacles": [{
                "cylinder_height": 750.0,
                "cylinder_radius": 300.0,
                "latitude": 38.140578,
                "longitude": -76.428997
            }, {
                "cylinder_height": 400.0,
                "cylinder_radius": 100.0,
                "latitude": 38.149156,
                "longitude": -76.430622
            }]
        }

        # Deserialize obstacles.
        args = (data, "odom", 1.0)
        moving, stationary = serializers.ObstaclesDeserializer.from_dict(*args)

        # Compare number of markers.
        self.assertEqual(len(data["moving_obstacles"]), len(moving.spheres))
        self.assertEqual(
            len(data["stationary_obstacles"]), len(stationary.cylinders))

        # Test moving obstacle properties.
        for i, sphere in enumerate(moving.spheres):
            obs = data["moving_obstacles"][i]

            altitude = serializers.feet_to_meters(obs["altitude_msl"])
            self.assertEqual(sphere.center.latitude, obs["latitude"])
            self.assertEqual(sphere.center.longitude, obs["longitude"])

            radius = serializers.feet_to_meters(obs["sphere_radius"])
            self.assertEqual(sphere.radius, radius)

        # Test stationary obstacle properties.
        for i, cylinder in enumerate(stationary.cylinders):
            obs = data["stationary_obstacles"][i]
            height = serializers.feet_to_meters(obs["cylinder_height"])

            self.assertEqual(cylinder.center.latitude, obs["latitude"])
            self.assertEqual(cylinder.center.longitude, obs["longitude"])

            radius = serializers.feet_to_meters(obs["cylinder_radius"])
            self.assertEqual(cylinder.radius, radius)
            self.assertEqual(cylinder.height, height)

    def test_telemetry_serializer(self):
        """Tests telemetry serializer."""
        # Set up test data.
        navsat = NavSatFix()
        navsat.latitude = 38.149
        navsat.longitude = -76.432
        navsat.altitude = 30.48
        compass = Float64(90.0)

        data = serializers.TelemetrySerializer.from_msg(navsat, compass)
        altitude_msl = serializers.meters_to_feet(navsat.altitude)

        # Compare.
        self.assertEqual(data["latitude"], navsat.latitude)
        self.assertEqual(data["longitude"], navsat.longitude)
        self.assertEqual(data["altitude_msl"], altitude_msl)
        self.assertEqual(data["uas_heading"], compass.data)

    def test_object_serializer(self):
        """Tests object serializer."""
        # Set up test data.
        object = Object()
        object.type.data = ObjectType.STANDARD
        object.latitude = 38.1478
        object.longitude = -76.4275
        object.orientation.data = Orientation.NORTH
        object.shape.data = Shape.STAR
        object.background_color.data = Color.ORANGE
        object.alphanumeric_color.data = Color.ORANGE
        object.alphanumeric = "C"
        object.description = ""
        object.autonomous = False

        # Serialize object message.
        data = serializers.ObjectSerializer.from_msg(object)

        # Compare.
        self.assertEqual(data["type"], object.type.data)
        self.assertEqual(data["latitude"], object.latitude)
        self.assertEqual(data["longitude"], object.longitude)
        self.assertEqual(data["orientation"], object.orientation.data)
        self.assertEqual(data["shape"], object.shape.data)
        self.assertEqual(data["background_color"], object.background_color.data)
        self.assertEqual(data["alphanumeric_color"],
                         object.alphanumeric_color.data)
        self.assertEqual(data["alphanumeric"], object.alphanumeric)
        self.assertEqual(data["description"], object.description)
        self.assertEqual(data["autonomous"], object.autonomous)

    def test_object_deserializer(self):
        """Tests object deserializer."""
        # Set up test data.
        data = {
            "id": 1,
            "user": 1,
            "type": "standard",
            "latitude": 38.1478,
            "longitude": -76.4275,
            "orientation": "n",
            "shape": "star",
            "background_color": "orange",
            "alphanumeric": "C",
            "alphanumeric_color": "black",
            "description": "",
            "autonomous": False
        }

        # Deserialize object data.
        object = serializers.ObjectSerializer.from_dict(data)

        # Compare.
        self.assertEqual(data["type"], object.type.data)
        self.assertEqual(data["latitude"], object.latitude)
        self.assertEqual(data["longitude"], object.longitude)
        self.assertEqual(data["orientation"], object.orientation.data)
        self.assertEqual(data["shape"], object.shape.data)
        self.assertEqual(data["background_color"], object.background_color.data)
        self.assertEqual(data["alphanumeric_color"],
                         object.alphanumeric_color.data)
        self.assertEqual(data["alphanumeric"], object.alphanumeric)
        self.assertEqual(data["description"], object.description)
        self.assertEqual(data["autonomous"], object.autonomous)

    def test_object_image_serializer(self):
        """Tests object image serializer can be deserialized."""
        # Create random 40 x 30 RGB image.
        width = 40
        height = 30
        nparr = np.random.randint(0, 256, (width, height, 3)).astype(np.uint8)

        # Convert to ROS Image.
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(nparr)

        # Serialize.
        png = serializers.ObjectImageSerializer.from_msg(msg)

        # Deserialize.
        converted_msg = serializers.ObjectImageSerializer.from_raw(png)

        # Convert to array.
        converted_img = bridge.imgmsg_to_cv2(converted_msg)
        converted_arr = np.asarray(converted_img)

        # Test if we get the original image.
        self.assertTrue((converted_arr == nparr).all())


if __name__ == "__main__":
    rospy.init_node("test_serializers")
    rosunit.unitrun("test_serializers", "test_serializers", TestSerializers)
