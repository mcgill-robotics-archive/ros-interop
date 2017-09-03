#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Interoperability serialization tests."""

import rospy
import rosunit
import numpy as np
from unittest import TestCase
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from interop.client import InteroperabilityClient
from mock_server import InteroperabilityMockServer
from interop.serializers import ObjectSerializer, ObjectImageSerializer


class TestInteroperabilityClient(TestCase):

    """Tests interoperability client."""

    def test_get_obstacles(self):
        """Tests getting obstacle data through client."""
        # Set up test data.
        url = "http://interop"
        client_args = (url, "testuser", "testpass", 1.0)
        json = {
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

        with InteroperabilityMockServer(url) as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_get_obstacles_response(json)

            # Connect client.
            client = InteroperabilityClient(*client_args)
            client.wait_for_server()
            client.login()
            client.get_obstacles("odom", 1.0)

    def test_post_telemetry(self):
        """Tests posting telemetry data through client."""
        # Set up test data.
        url = "http://interop"
        client_args = (url, "testuser", "testpass", 1.0)

        with InteroperabilityMockServer(url) as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_telemetry_response()

            # Connect client.
            client = InteroperabilityClient(*client_args)
            client.wait_for_server()
            client.login()
            client.post_telemetry(NavSatFix(), Float64())

    def test_post_object(self):
        """Tests posting object data through client."""
        # Set up test data.
        url = "http://interop"
        client_args = (url, "testuser", "testpass", 1.0)
        data = {
            "type": "standard",
            "latitude": 38.1478,
            "longitude": -76.4275,
            "orientation": "n",
            "shape": "star",
            "background_color": "orange",
            "alphanumeric": "C",
            "alphanumeric_color": "black"
        }

        with InteroperabilityMockServer(url) as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_post_object_response(data, id=1)

            # Connect client.
            client = InteroperabilityClient(*client_args)
            client.wait_for_server()
            client.login()
            client.post_object(data)

    def test_objects(self):
        """Tests posting, updating, retrieving and deleting object data
        through client."""
        # Set up test data.
        url = "http://interop"
        client_args = (url, "testuser", "testpass", 1.0)
        objects = [{
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
            "description": None,
            "autonomous": False
        }, {
            "id": 2,
            "user": 1,
            "type": "qrc",
            "latitude": 38.1878,
            "longitude": -76.4075,
            "orientation": None,
            "shape": None,
            "background_color": None,
            "alphanumeric": None,
            "alphanumeric_color": None,
            "description": "http://auvsi-seafarer.org",
            "autonomous": False
        }]

        with InteroperabilityMockServer(url) as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_get_objects_response(objects)

            # Connect client.
            client = InteroperabilityClient(*client_args)
            client.wait_for_server()
            client.login()
            all_objects = client.get_all_objects()

            # Compare all objects.
            for t in objects:
                # Verify ID is correct.
                self.assertTrue(t["id"] in all_objects)

                # Get object individually.
                curr_object = client.get_object(t["id"])

                # Try to update the objects.
                server.set_put_object_response(t["id"], curr_object)
                client.put_object(t["id"], curr_object)

                # Try to delete object.
                server.set_delete_object_response(t["id"])
                client.delete_object(t["id"])

    def test_object_image(self):
        """Tests posting telemetry data through client."""
        # Set up test data.
        url = "http://interop"
        client_args = (url, "testuser", "testpass", 1.0)
        object_id = 1

        width = 40
        height = 30
        nparr = np.random.randint(0, 256, (width, height, 3)).astype(np.uint8)

        bridge = CvBridge()
        ros_img = bridge.cv2_to_imgmsg(nparr)

        img = ObjectImageSerializer.from_msg(ros_img)

        with InteroperabilityMockServer(url) as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_post_object_image_response(object_id)
            server.set_get_object_image_response(object_id, img, "image/png")
            server.set_delete_object_image_response(object_id)

            # Connect client.
            client = InteroperabilityClient(*client_args)
            client.wait_for_server()
            client.login()
            client.post_object_image(object_id, ros_img)
            client.get_object_image(object_id)
            client.delete_object_image(object_id)


if __name__ == "__main__":
    rospy.init_node("test_client")
    rosunit.unitrun("test_client", "test_client", TestInteroperabilityClient)
