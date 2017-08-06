#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Test local object file writing and syncing."""

import unittest
import io
import shutil
import os.path
import json
import errno
import rospy
import rosunit
import numpy as np
from PIL import Image
from interop.client import InteroperabilityClient
from mock_server import InteroperabilityMockServer
from interop.local_targets import Target


def generate_image():
    """Generate a random 40 x 30 RGB PNG image.

    Returns:
        bytes: The PNG image.
    """
    # Generate random pixels.
    np_array = np.random.randint(0, 255, size=(1200, 3))
    pixel_list = [tuple(pixel) for pixel in np_array]

    # Create the PNG image.
    output = io.BytesIO()
    image = Image.new("RGB", (30, 40))
    image.putdata(pixel_list)
    image.save(output, format="PNG")

    return output.getvalue()


class TestTarget(unittest.TestCase):

    """Tests local object file writing, updating, and deleting.
    Tests syncing to the interop server.
    """

    def setUp(self):
        """Creates a directory for targets and images, sets up the client,
        and creates a Target. This is run before each test.
        """
        # Create the targets directory.
        self.targets_dir = "/tmp/interop_test/"
        try:
            os.mkdir(self.targets_dir)
        except OSError as e:
            if e.errno == errno.EEXIST:
                # Replace the existing directory
                shutil.rmtree(self.targets_dir)
                os.mkdir(self.targets_dir)
            else:
                raise

        # Set up the client.
        self.client = InteroperabilityClient("http://interop", "testuser",
                                             "testpass", 1.0)

        # Create a Target.
        self.target_data = {
            "type": "standard",
            "latitude": 38.1478,
            "longitude": -76.4275,
            "orientation": "n",
            "shape": "star",
            "background_color": "orange",
            "alphanumeric": "C",
            "alphanumeric_color": "black",
            "autonomous": False
        }

        json_data = json.dumps(self.target_data)
        self.target = Target(self.targets_dir, 1, json_data, self.client)

    def tearDown(self):
        """Cleans up after each test. Removes the targets directory."""
        shutil.rmtree(self.targets_dir)

    def test_add_target(self):
        """Tests that the adding of a Target has worked in setUp."""
        # Check that the file contents are the same as what was put in.
        with open(os.path.join(self.targets_dir, "1.json"), "r") as f:
            file_contents = f.read()

        self.assertEqual(json.loads(file_contents), self.target_data)

    def test_get_target(self):
        """Tests the retrieval of a Target."""
        self.assertEqual(json.loads(self.target.get()), self.target_data)

    def test_update_target(self):
        """Tests that updating of a Target."""
        # Update target
        updated_target = self.target_data.copy()
        updated_target["shape"] = "circle"
        json_target = json.dumps(updated_target)
        self.target.update(json_target)

        self.assertEqual(json.loads(self.target.get()), updated_target)

    def test_delete_target(self):
        """Tests the deletion of a Target."""
        self.target.delete()

        # Check that the file does not exist.
        self.assertFalse(
            os.path.exists(os.path.join(self.targets_dir, "1.json")))

        # Try to delete a previously deleted file.
        self.assertRaises(IOError, self.target.delete)

    def test_set_image(self):
        """Tests the setting of an image."""
        image = generate_image()
        self.target.set_image(image)

        # Check that the expected image file exists.
        self.assertTrue(os.path.exists(os.path.join(self.targets_dir, "1.png")))

        # Check that the retrieved data is the same as what was put in.
        with open(os.path.join(self.targets_dir, "1.png"), "r") as f:
            saved_image = f.read()

        self.assertEqual(saved_image, image)

    def test_get_image(self):
        """Tests the retrieval of an image associated with a Target."""
        image = generate_image()
        self.target.set_image(image)

        self.assertEqual(self.target.get_image(), image)

    def test_delete_image(self):
        """Tests the deletion of an image."""
        self.target.set_image(generate_image())  # Set an image.
        self.target.delete_image()  # Delete the image.

        # Check that the image does not exist after deleting.
        self.assertFalse(
            os.path.exists(os.path.join(self.targets_dir, "1.png")))

        # Try to delete a previously deleted image.
        self.assertRaises(OSError, self.target.delete_image)

    def test_initial_state_variables(self):
        """Tests the state variables right after a target is added."""
        self.assertTrue(self.target._needs_adding)
        self.assertFalse(self.target._needs_updating)
        self.assertFalse(self.target._needs_deleting)
        self.assertFalse(self.target._image_needs_setting)
        self.assertFalse(self.target._image_needs_deleting)

        self.assertIsNone(self.target.interop_id)

    def test_state_variables_for_unnecessary_updates_and_deletes_before_sync(
            self):
        """Tests that the Target class recognizes that there is no need to update
        or delete on the interop server if the target has not yet been uploaded
        onto the server.
        """
        # Update target before a sync and check that no update is set.
        updated_target = self.target_data.copy()
        updated_target["shape"] = "circle"
        json_target = json.dumps(updated_target)
        self.target.update(json_target)

        self.assertTrue(self.target._needs_adding)
        self.assertFalse(self.target._needs_updating)
        self.assertFalse(self.target._needs_deleting)
        self.assertFalse(self.target._image_needs_setting)
        self.assertFalse(self.target._image_needs_deleting)

        # Delete target before a sync and check that no delete is set.
        self.target.delete()
        self.assertFalse(self.target._needs_adding)
        self.assertFalse(self.target._needs_updating)
        self.assertFalse(self.target._needs_deleting)
        self.assertFalse(self.target._image_needs_setting)
        self.assertFalse(self.target._image_needs_deleting)

    def test_state_variables_for_image(self):
        """Tests state variables after setting and deleting a target image."""
        # Test the setting of an image.
        self.target.set_image(generate_image())

        self.assertTrue(self.target._needs_adding)
        self.assertFalse(self.target._needs_updating)
        self.assertFalse(self.target._needs_deleting)
        self.assertTrue(self.target._image_needs_setting)
        self.assertFalse(self.target._image_needs_deleting)

        # Test the deletion of an image.
        self.target.delete_image()

        self.assertTrue(self.target._needs_adding)
        self.assertFalse(self.target._needs_updating)
        self.assertFalse(self.target._needs_deleting)
        self.assertFalse(self.target._image_needs_setting)
        self.assertFalse(self.target._image_needs_deleting)

        # Test that nothing needs to be done to the image when the target is
        # deleted.
        self.target.set_image(generate_image())
        self.target.delete()

        self.assertFalse(self.target._needs_adding)
        self.assertFalse(self.target._needs_updating)
        self.assertFalse(self.target._needs_deleting)
        self.assertFalse(self.target._image_needs_setting)
        self.assertFalse(self.target._image_needs_deleting)

        # Check image_is_on_server.
        self.assertFalse(self.target.image_is_on_server)

    def test_state_variables_after_successful_sync(self):
        """Tests that the state variables change to reflect successful syncs."""
        # Immediately sync the new target.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_post_target_response(self.target_data, 1)

            self.client.wait_for_server()
            self.client.login()
            self.target.sync()

        self.assertFalse(self.target._needs_adding)
        self.assertFalse(self.target._needs_updating)
        self.assertFalse(self.target._needs_deleting)
        self.assertFalse(self.target._image_needs_setting)
        self.assertFalse(self.target._image_needs_deleting)

        self.assertEqual(self.target.interop_id, 1)

        # Update the target.
        updated_target = self.target_data.copy()
        updated_target["shape"] = "circle"
        json_target = json.dumps(updated_target)
        self.target.update(json_target)

        self.assertFalse(self.target._needs_adding)
        self.assertTrue(self.target._needs_updating)
        self.assertFalse(self.target._needs_deleting)
        self.assertFalse(self.target._image_needs_setting)
        self.assertFalse(self.target._image_needs_deleting)

        self.assertEqual(self.target.interop_id, 1)

        # Update the target on the server.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_put_target_response(1, updated_target)

            self.client.wait_for_server()
            self.client.login()
            self.target.sync()

        self.assertFalse(self.target._needs_adding)
        self.assertFalse(self.target._needs_updating)
        self.assertFalse(self.target._needs_deleting)
        self.assertFalse(self.target._image_needs_setting)
        self.assertFalse(self.target._image_needs_deleting)

        # Set an image.
        self.target.set_image(generate_image())

        self.assertFalse(self.target._needs_adding)
        self.assertFalse(self.target._needs_updating)
        self.assertFalse(self.target._needs_deleting)
        self.assertTrue(self.target._image_needs_setting)
        self.assertFalse(self.target._image_needs_deleting)

        # Set the image on the server.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_post_target_image_response(1)

            self.client.wait_for_server()
            self.client.login()
            self.target.sync()

        self.assertFalse(self.target._needs_adding)
        self.assertFalse(self.target._needs_updating)
        self.assertFalse(self.target._needs_deleting)
        self.assertFalse(self.target._image_needs_setting)
        self.assertFalse(self.target._image_needs_deleting)

        self.assertTrue(self.target.image_is_on_server)

        # Delete the image.
        self.target.delete_image()

        self.assertFalse(self.target._needs_adding)
        self.assertFalse(self.target._needs_updating)
        self.assertFalse(self.target._needs_deleting)
        self.assertFalse(self.target._image_needs_setting)
        self.assertTrue(self.target._image_needs_deleting)

        # Delete the image on the server.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_delete_target_image_response(1)

            self.client.wait_for_server()
            self.client.login()
            self.target.sync()

        self.assertFalse(self.target._needs_adding)
        self.assertFalse(self.target._needs_updating)
        self.assertFalse(self.target._needs_deleting)
        self.assertFalse(self.target._image_needs_setting)
        self.assertFalse(self.target._image_needs_deleting)

        self.assertFalse(self.target.image_is_on_server)

        # Set an image again, then sync the image, then delete the target.
        self.target.set_image(generate_image())

        # Set the image on the server.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_post_target_image_response(1)

            self.client.wait_for_server()
            self.client.login()
            self.target.sync()

        # Delete the target.
        self.target.delete()

        self.assertFalse(self.target._needs_adding)
        self.assertFalse(self.target._needs_updating)
        self.assertTrue(self.target._needs_deleting)
        self.assertFalse(self.target._image_needs_setting)
        self.assertFalse(self.target._image_needs_deleting)

        # Delete the target on the server.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_delete_target_response(1)

            self.client.wait_for_server()
            self.client.login()
            self.target.sync()

        self.assertFalse(self.target._needs_adding)
        self.assertFalse(self.target._needs_updating)
        self.assertFalse(self.target._needs_deleting)
        self.assertFalse(self.target._image_needs_setting)
        self.assertFalse(self.target._image_needs_deleting)

        self.assertIsNone(self.target.interop_id)
        self.assertFalse(self.target.image_is_on_server)

    def test_state_variables_after_unsuccessful_target_sync(self):
        """Tests that the state variables reflect unsuccessful target syncs."""
        # Immediately sync the new target.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()

            # Login.
            self.client.wait_for_server()
            self.client.login()

            # Test unsuccessful add on server.
            # Try to add target to server.
            codes = [400, 404, 500]
            for code in codes:
                server.set_post_target_response(self.target_data, 1, code=code)
                self.target.sync()

                self.assertTrue(self.target._needs_adding)
                self.assertFalse(self.target._needs_updating)
                self.assertFalse(self.target._needs_deleting)
                self.assertFalse(self.target._image_needs_setting)
                self.assertFalse(self.target._image_needs_deleting)

                self.assertIsNone(self.target.interop_id)

        # Successfully add, then test unsuccessful update and delete.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()

            # Login.
            self.client.wait_for_server()
            self.client.login()

            # Successfully add target to server.
            server.set_post_target_response(self.target_data, 1)
            self.target.sync()

            # Test unsuccessful update on server.
            # Update target.
            updated_target = self.target_data.copy()
            updated_target["shape"] = "circle"
            self.target.update(json.dumps(updated_target))

            # Try to update on server.
            codes = [400, 404, 500]
            for code in codes:
                server.set_put_target_response(1, self.target_data, code=code)
                self.target.sync()

                self.assertFalse(self.target._needs_adding)
                self.assertTrue(self.target._needs_updating)
                self.assertFalse(self.target._needs_deleting)
                self.assertFalse(self.target._image_needs_setting)
                self.assertFalse(self.target._image_needs_deleting)

            # Delete target.
            self.target.delete()

            # Test unsuccessful delete on server.
            # Try to delete target from server.
            for code in codes:
                server.set_delete_target_response(1, code=code)
                self.target.sync()

                self.assertFalse(self.target._needs_adding)
                self.assertFalse(self.target._needs_updating)
                self.assertTrue(self.target._needs_deleting)
                self.assertFalse(self.target._image_needs_setting)
                self.assertFalse(self.target._image_needs_deleting)

    def test_state_variables_after_unsuccessful_image_sync(self):
        """Tests that the state variables reflect unsuccessful target image
        syncs.
        """
        # Successfully add the target.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_post_target_response(self.target_data, 1)

            self.client.wait_for_server()
            self.client.login()
            self.target.sync()

        # Test the unsuccessful setting of an image on the server.
        # Set an image.
        self.target.set_image(generate_image())

        # Try to set the image on the server.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()

            # Login
            self.client.wait_for_server()
            self.client.login()

            codes = [400, 404, 500]
            for code in codes:
                server.set_post_target_image_response(1, code=code)
                self.target.sync()

                self.assertFalse(self.target._needs_adding)
                self.assertFalse(self.target._needs_updating)
                self.assertFalse(self.target._needs_deleting)
                self.assertTrue(self.target._image_needs_setting)
                self.assertFalse(self.target._image_needs_deleting)

                self.assertFalse(self.target.image_is_on_server)

        # Successfully set the image on the server, then unsuccessfully delete
        # the image from the server.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()

            # Login.
            self.client.wait_for_server()
            self.client.login()

            # Test unsuccessful delete of image from server.
            # Set an image on the server, then try to delete the image from
            # the server.
            self.target.set_image(generate_image())
            server.set_post_target_image_response(1)
            self.target.sync()

            # Delete the image.
            self.target.delete_image()

            # Try to delete the image from the server.
            codes = [400, 404, 500]
            for code in codes:
                server.set_delete_target_image_response(1, code=code)
                self.target.sync()

                self.assertFalse(self.target._needs_adding)
                self.assertFalse(self.target._needs_updating)
                self.assertFalse(self.target._needs_deleting)
                self.assertFalse(self.target._image_needs_setting)
                self.assertTrue(self.target._image_needs_deleting)

                self.assertTrue(self.target.image_is_on_server)


if __name__ == "__main__":
    rospy.init_node("test_local_targets")
    rosunit.unitrun("test_local_targets", "test_target", TestTarget)
