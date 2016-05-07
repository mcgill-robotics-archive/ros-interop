#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Interoperability Serialization Tests."""

import utm
import rospy
import rosunit
import numpy as np
from unittest import TestCase
from cv_bridge import CvBridge
from interop import serializers
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker
from interop.msg import Color, Orientation, Shape, Target, TargetType


class TestSerializers(TestCase):

    """Tests interoperability serializers."""

    def test_server_info_deserializer(self):
        """Tests server information deserializer."""
        # Set up test data.
        json = {
            "message": "Fly Safe",
            "message_timestamp": "2015-06-14 18:18:55.642000+00:00",
            "server_time": "2015-08-14 03:37:13.331402"
        }

        # Deserialize server information.
        server_info = serializers.ServerInfoDeserializer.from_json(json)

        # Compare properties
        self.assertEqual(server_info[0].data, json["message"])
        msg_time = serializers.iso8601_to_rostime(json["message_timestamp"])
        self.assertEqual(server_info[1], msg_time)
        server_time = serializers.iso8601_to_rostime(json["server_time"])
        self.assertEqual(server_info[2], server_time)

    def test_obstacles_deserializer(self):
        """Tests obstacles deserializer."""
        # Set up test data.
        json = {
            "moving_obstacles": [
                {
                    "altitude_msl": 189.56748784643966,
                    "latitude": 38.141826869853645,
                    "longitude": -76.43199876559223,
                    "sphere_radius": 150.0
                },
                {
                    "altitude_msl": 250.0,
                    "latitude": 38.14923628783763,
                    "longitude": -76.43238529543882,
                    "sphere_radius": 150.0
                }
            ],
            "stationary_obstacles": [
                {
                    "cylinder_height": 750.0,
                    "cylinder_radius": 300.0,
                    "latitude": 38.140578,
                    "longitude": -76.428997
                },
                {
                    "cylinder_height": 400.0,
                    "cylinder_radius": 100.0,
                    "latitude": 38.149156,
                    "longitude": -76.430622
                }
            ]
        }

        # Deserialize obstacles.
        args = (json, "odom", 1.0)
        moving, stationary = serializers.ObstaclesDeserializer.from_json(*args)

        # Compare number of markers.
        self.assertEqual(len(json["moving_obstacles"]), len(moving.markers))
        self.assertEqual(len(json["stationary_obstacles"]),
                         len(stationary.markers))

        # Test moving obstacle properties.
        for i, marker in enumerate(moving.markers):
            self.assertEqual(marker.type, Marker.SPHERE)
            self.assertEqual(marker.ns, "moving_obstacles")

            obs = json["moving_obstacles"][i]
            altitude = serializers.feet_to_meters(obs["altitude_msl"])
            easting, northing, _, _ = utm.from_latlon(obs["latitude"],
                                                      obs["longitude"])
            self.assertEqual(marker.pose.position.x, easting)
            self.assertEqual(marker.pose.position.y, northing)
            self.assertEqual(marker.pose.position.z, altitude)

            radius = serializers.feet_to_meters(obs["sphere_radius"])
            self.assertEqual(marker.scale.x, radius)
            self.assertEqual(marker.scale.y, radius)
            self.assertEqual(marker.scale.z, radius)

        # Test stationary obstacle properties.
        for i, marker in enumerate(stationary.markers):
            self.assertEqual(marker.type, Marker.CYLINDER)
            self.assertEqual(marker.ns, "stationary_obstacles")

            obs = json["stationary_obstacles"][i]
            height = serializers.feet_to_meters(obs["cylinder_height"])
            easting, northing, _, _ = utm.from_latlon(obs["latitude"],
                                                      obs["longitude"])
            self.assertEqual(marker.pose.position.x, easting)
            self.assertEqual(marker.pose.position.y, northing)
            self.assertEqual(marker.pose.position.z, height / 2)

            radius = serializers.feet_to_meters(obs["cylinder_radius"])
            self.assertEqual(marker.scale.x, radius)
            self.assertEqual(marker.scale.y, radius)
            self.assertEqual(marker.scale.z, height)

    def test_telemetry_serializer(self):
        """Tests telemetry serializer."""
        # Set up test data.
        navsat = NavSatFix()
        navsat.latitude = 38.149
        navsat.longitude = -76.432
        navsat.altitude = 30.48
        compass = Float64(90.0)

        json = serializers.TelemetrySerializer.from_msg(navsat, compass)
        altitude_msl = serializers.meters_to_feet(navsat.altitude)

        # Compare.
        self.assertEqual(json["latitude"], navsat.latitude)
        self.assertEqual(json["longitude"], navsat.longitude)
        self.assertEqual(json["altitude_msl"], altitude_msl)
        self.assertEqual(json["uas_heading"], compass.data)

    def test_target_serializer(self):
        """Tests target serializer."""
        # Set up test data.
        target = Target()
        target.type.data = TargetType.STANDARD
        target.latitude = 38.1478
        target.longitude = -76.4275
        target.orientation.data = Orientation.NORTH
        target.shape.data = Shape.STAR
        target.background_color.data = Color.ORANGE
        target.alphanumeric_color.data = Color.ORANGE
        target.alphanumeric = "C"
        target.description = ""
        target.autonomous = False

        # Serialize target message.
        json = serializers.TargetSerializer.from_msg(target)

        # Compare.
        self.assertEqual(json["type"], target.type.data)
        self.assertEqual(json["latitude"], target.latitude)
        self.assertEqual(json["longitude"], target.longitude)
        self.assertEqual(json["orientation"], target.orientation.data)
        self.assertEqual(json["shape"], target.shape.data)
        self.assertEqual(json["background_color"],
                         target.background_color.data)
        self.assertEqual(json["alphanumeric_color"],
                         target.alphanumeric_color.data)
        self.assertEqual(json["alphanumeric"], target.alphanumeric)
        self.assertEqual(json["description"], target.description)
        self.assertEqual(json["autonomous"], target.autonomous)

    def test_target_deserializer(self):
        """Tests target deserializer."""
        # Set up test data.
        json = {
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

        # Deserialize target JSON data.
        target = serializers.TargetSerializer.from_json(json)

        # Compare.
        self.assertEqual(json["type"], target.type.data)
        self.assertEqual(json["latitude"], target.latitude)
        self.assertEqual(json["longitude"], target.longitude)
        self.assertEqual(json["orientation"], target.orientation.data)
        self.assertEqual(json["shape"], target.shape.data)
        self.assertEqual(json["background_color"],
                         target.background_color.data)
        self.assertEqual(json["alphanumeric_color"],
                         target.alphanumeric_color.data)
        self.assertEqual(json["alphanumeric"], target.alphanumeric)
        self.assertEqual(json["description"], target.description)
        self.assertEqual(json["autonomous"], target.autonomous)

    def test_target_image_serializer(self):
        """Tests target image serializer can be deserialized."""
        # Create random 40 x 30 RGB image.
        width = 40
        height = 30
        nparr = np.random.randint(0, 256, (width, height, 3)).astype(np.uint8)

        # Convert to ROS Image.
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(nparr)

        # Serialize.
        png = serializers.TargetImageSerializer.from_msg(msg)

        # Deserialize.
        converted_msg = serializers.TargetImageSerializer.from_raw(png)

        # Convert to array.
        converted_img = bridge.imgmsg_to_cv2(converted_msg)
        converted_arr = np.asarray(converted_img)

        # Test if we get the original image.
        self.assertTrue((converted_arr == nparr).all())


if __name__ == "__main__":
    rospy.init_node("test_serializers")
    rosunit.unitrun("test_serializers", "test_serializers", TestSerializers)
