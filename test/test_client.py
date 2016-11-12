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
from interop.serializers import TargetSerializer, TargetImageSerializer


class TestInteroperabilityClient(TestCase):

    """Tests interoperability client."""

    def test_get_obstacles(self):
        """Tests getting obstacle data through client."""
        # Set up test data.
        url = "http://interop"
        client_args = (url, "testuser", "testpass", 1.0)
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

    def test_post_target(self):
        """Tests posting target data through client."""
        # Set up test data.
        url = "http://interop"
        client_args = (url, "testuser", "testpass", 1.0)
        target_json = {
            "type": "standard",
            "latitude": 38.1478,
            "longitude": -76.4275,
            "orientation": "n",
            "shape": "star",
            "background_color": "orange",
            "alphanumeric": "C",
            "alphanumeric_color": "black"
        }
        target = TargetSerializer.from_json(target_json)

        with InteroperabilityMockServer(url) as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_post_target_response(target, id=1)

            # Connect client.
            client = InteroperabilityClient(*client_args)
            client.wait_for_server()
            client.login()
            client.post_target(target)

    def test_targets(self):
        """Tests posting, updating, retrieving and deleting target data
        through client."""
        # Set up test data.
        url = "http://interop"
        client_args = (url, "testuser", "testpass", 1.0)
        targets = [
            {
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
            },
            {
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
            }
        ]

        with InteroperabilityMockServer(url) as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_get_targets_response(targets)

            # Connect client.
            client = InteroperabilityClient(*client_args)
            client.wait_for_server()
            client.login()
            all_targets = client.get_all_targets()

            # Compare all targets.
            for t in targets:
                # Verify ID is correct.
                self.assertTrue(t["id"] in all_targets)

                # Get target individually.
                curr_target = client.get_target(t["id"])

                # Try to update the targets.
                server.set_put_target_response(t["id"], curr_target)
                client.put_target(t["id"], curr_target)

                # Try to delete target.
                server.set_delete_target_response(t["id"])
                client.delete_target(t["id"])

    def test_target_image(self):
        """Tests posting telemetry data through client."""
        # Set up test data.
        url = "http://interop"
        client_args = (url, "testuser", "testpass", 1.0)
        target_id = 1

        width = 40
        height = 30
        nparr = np.random.randint(0, 256, (width, height, 3)).astype(np.uint8)

        bridge = CvBridge()
        ros_img = bridge.cv2_to_imgmsg(nparr)

        img = TargetImageSerializer.from_msg(ros_img)

        with InteroperabilityMockServer(url) as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_post_target_image_response(target_id)
            server.set_get_target_image_response(target_id, img, "image/png")
            server.set_delete_target_image_response(target_id)

            # Connect client.
            client = InteroperabilityClient(*client_args)
            client.wait_for_server()
            client.login()
            client.post_target_image(target_id, ros_img)
            client.get_target_image(target_id)
            client.delete_target_image(target_id)


if __name__ == "__main__":
    rospy.init_node("test_client")
    rosunit.unitrun("test_client", "test_client", TestInteroperabilityClient)
