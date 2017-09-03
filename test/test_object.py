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
from interop.local_objects import Object


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


class TestObject(unittest.TestCase):

    """Tests local object file writing, updating, and deleting.
    Tests syncing to the interop server.
    """

    def setUp(self):
        """Creates a directory for objects and images, sets up the client,
        and creates an Object. This is run before each test.
        """
        # Create the objects directory.
        self.objects_dir = "/tmp/interop_test/"
        try:
            os.mkdir(self.objects_dir)
        except OSError as e:
            if e.errno == errno.EEXIST:
                # Replace the existing directory
                shutil.rmtree(self.objects_dir)
                os.mkdir(self.objects_dir)
            else:
                raise

        # Set up the client.
        self.client = InteroperabilityClient("http://interop", "testuser",
                                             "testpass", 1.0)

        # Create an Object.
        self.object_data = {
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

        json_data = json.dumps(self.object_data)
        self.object = Object(self.objects_dir, 1, json_data, self.client)

    def tearDown(self):
        """Cleans up after each test. Removes the objects directory."""
        shutil.rmtree(self.objects_dir)

    def test_add_object(self):
        """Tests that the adding of an Object has worked in setUp."""
        # Check that the file contents are the same as what was put in.
        with open(os.path.join(self.objects_dir, "1.json"), "r") as f:
            file_contents = f.read()

        self.assertEqual(json.loads(file_contents), self.object_data)

    def test_get_object(self):
        """Tests the retrieval of an Object."""
        self.assertEqual(json.loads(self.object.get()), self.object_data)

    def test_update_object(self):
        """Tests that updating of an Object."""
        # Update object
        updated_object = self.object_data.copy()
        updated_object["shape"] = "circle"
        json_object = json.dumps(updated_object)
        self.object.update(json_object)

        self.assertEqual(json.loads(self.object.get()), updated_object)

    def test_delete_object(self):
        """Tests the deletion of an Object."""
        self.object.delete()

        # Check that the file does not exist.
        self.assertFalse(
            os.path.exists(os.path.join(self.objects_dir, "1.json")))

        # Try to delete a previously deleted file.
        self.assertRaises(IOError, self.object.delete)

    def test_set_image(self):
        """Tests the setting of an image."""
        image = generate_image()
        self.object.set_image(image)

        # Check that the expected image file exists.
        self.assertTrue(os.path.exists(os.path.join(self.objects_dir, "1.png")))

        # Check that the retrieved data is the same as what was put in.
        with open(os.path.join(self.objects_dir, "1.png"), "r") as f:
            saved_image = f.read()

        self.assertEqual(saved_image, image)

    def test_get_image(self):
        """Tests the retrieval of an image associated with an Object."""
        image = generate_image()
        self.object.set_image(image)

        self.assertEqual(self.object.get_image(), image)

    def test_delete_image(self):
        """Tests the deletion of an image."""
        self.object.set_image(generate_image())  # Set an image.
        self.object.delete_image()  # Delete the image.

        # Check that the image does not exist after deleting.
        self.assertFalse(
            os.path.exists(os.path.join(self.objects_dir, "1.png")))

        # Try to delete a previously deleted image.
        self.assertRaises(OSError, self.object.delete_image)

    def test_initial_state_variables(self):
        """Tests the state variables right after an object is added."""
        self.assertTrue(self.object._needs_adding)
        self.assertFalse(self.object._needs_updating)
        self.assertFalse(self.object._needs_deleting)
        self.assertFalse(self.object._image_needs_setting)
        self.assertFalse(self.object._image_needs_deleting)

        self.assertIsNone(self.object.interop_id)

    def test_state_variables_for_unnecessary_updates_and_deletes_before_sync(
            self):
        """Tests that the Object class recognizes that there is no need to update
        or delete on the interop server if the object has not yet been uploaded
        onto the server.
        """
        # Update object before a sync and check that no update is set.
        updated_object = self.object_data.copy()
        updated_object["shape"] = "circle"
        json_object = json.dumps(updated_object)
        self.object.update(json_object)

        self.assertTrue(self.object._needs_adding)
        self.assertFalse(self.object._needs_updating)
        self.assertFalse(self.object._needs_deleting)
        self.assertFalse(self.object._image_needs_setting)
        self.assertFalse(self.object._image_needs_deleting)

        # Delete object before a sync and check that no delete is set.
        self.object.delete()
        self.assertFalse(self.object._needs_adding)
        self.assertFalse(self.object._needs_updating)
        self.assertFalse(self.object._needs_deleting)
        self.assertFalse(self.object._image_needs_setting)
        self.assertFalse(self.object._image_needs_deleting)

    def test_state_variables_for_image(self):
        """Tests state variables after setting and deleting an object image."""
        # Test the setting of an image.
        self.object.set_image(generate_image())

        self.assertTrue(self.object._needs_adding)
        self.assertFalse(self.object._needs_updating)
        self.assertFalse(self.object._needs_deleting)
        self.assertTrue(self.object._image_needs_setting)
        self.assertFalse(self.object._image_needs_deleting)

        # Test the deletion of an image.
        self.object.delete_image()

        self.assertTrue(self.object._needs_adding)
        self.assertFalse(self.object._needs_updating)
        self.assertFalse(self.object._needs_deleting)
        self.assertFalse(self.object._image_needs_setting)
        self.assertFalse(self.object._image_needs_deleting)

        # Test that nothing needs to be done to the image when the object is
        # deleted.
        self.object.set_image(generate_image())
        self.object.delete()

        self.assertFalse(self.object._needs_adding)
        self.assertFalse(self.object._needs_updating)
        self.assertFalse(self.object._needs_deleting)
        self.assertFalse(self.object._image_needs_setting)
        self.assertFalse(self.object._image_needs_deleting)

        # Check image_is_on_server.
        self.assertFalse(self.object.image_is_on_server)

    def test_state_variables_after_successful_sync(self):
        """Tests that the state variables change to reflect successful syncs."""
        # Immediately sync the new object.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_post_object_response(self.object_data, 1)

            self.client.wait_for_server()
            self.client.login()
            self.object.sync()

        self.assertFalse(self.object._needs_adding)
        self.assertFalse(self.object._needs_updating)
        self.assertFalse(self.object._needs_deleting)
        self.assertFalse(self.object._image_needs_setting)
        self.assertFalse(self.object._image_needs_deleting)

        self.assertEqual(self.object.interop_id, 1)

        # Update the object.
        updated_object = self.object_data.copy()
        updated_object["shape"] = "circle"
        json_object = json.dumps(updated_object)
        self.object.update(json_object)

        self.assertFalse(self.object._needs_adding)
        self.assertTrue(self.object._needs_updating)
        self.assertFalse(self.object._needs_deleting)
        self.assertFalse(self.object._image_needs_setting)
        self.assertFalse(self.object._image_needs_deleting)

        self.assertEqual(self.object.interop_id, 1)

        # Update the object on the server.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_put_object_response(1, updated_object)

            self.client.wait_for_server()
            self.client.login()
            self.object.sync()

        self.assertFalse(self.object._needs_adding)
        self.assertFalse(self.object._needs_updating)
        self.assertFalse(self.object._needs_deleting)
        self.assertFalse(self.object._image_needs_setting)
        self.assertFalse(self.object._image_needs_deleting)

        # Set an image.
        self.object.set_image(generate_image())

        self.assertFalse(self.object._needs_adding)
        self.assertFalse(self.object._needs_updating)
        self.assertFalse(self.object._needs_deleting)
        self.assertTrue(self.object._image_needs_setting)
        self.assertFalse(self.object._image_needs_deleting)

        # Set the image on the server.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_post_object_image_response(1)

            self.client.wait_for_server()
            self.client.login()
            self.object.sync()

        self.assertFalse(self.object._needs_adding)
        self.assertFalse(self.object._needs_updating)
        self.assertFalse(self.object._needs_deleting)
        self.assertFalse(self.object._image_needs_setting)
        self.assertFalse(self.object._image_needs_deleting)

        self.assertTrue(self.object.image_is_on_server)

        # Delete the image.
        self.object.delete_image()

        self.assertFalse(self.object._needs_adding)
        self.assertFalse(self.object._needs_updating)
        self.assertFalse(self.object._needs_deleting)
        self.assertFalse(self.object._image_needs_setting)
        self.assertTrue(self.object._image_needs_deleting)

        # Delete the image on the server.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_delete_object_image_response(1)

            self.client.wait_for_server()
            self.client.login()
            self.object.sync()

        self.assertFalse(self.object._needs_adding)
        self.assertFalse(self.object._needs_updating)
        self.assertFalse(self.object._needs_deleting)
        self.assertFalse(self.object._image_needs_setting)
        self.assertFalse(self.object._image_needs_deleting)

        self.assertFalse(self.object.image_is_on_server)

        # Set an image again, then sync the image, then delete the object.
        self.object.set_image(generate_image())

        # Set the image on the server.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_post_object_image_response(1)

            self.client.wait_for_server()
            self.client.login()
            self.object.sync()

        # Delete the object.
        self.object.delete()

        self.assertFalse(self.object._needs_adding)
        self.assertFalse(self.object._needs_updating)
        self.assertTrue(self.object._needs_deleting)
        self.assertFalse(self.object._image_needs_setting)
        self.assertFalse(self.object._image_needs_deleting)

        # Delete the object on the server.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_delete_object_response(1)

            self.client.wait_for_server()
            self.client.login()
            self.object.sync()

        self.assertFalse(self.object._needs_adding)
        self.assertFalse(self.object._needs_updating)
        self.assertFalse(self.object._needs_deleting)
        self.assertFalse(self.object._image_needs_setting)
        self.assertFalse(self.object._image_needs_deleting)

        self.assertIsNone(self.object.interop_id)
        self.assertFalse(self.object.image_is_on_server)

    def test_state_variables_after_unsuccessful_object_sync(self):
        """Tests that the state variables reflect unsuccessful object syncs."""
        # Immediately sync the new object.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()

            # Login.
            self.client.wait_for_server()
            self.client.login()

            # Test unsuccessful add on server.
            # Try to add object to server.
            codes = [400, 404, 500]
            for code in codes:
                server.set_post_object_response(self.object_data, 1, code=code)
                self.object.sync()

                self.assertTrue(self.object._needs_adding)
                self.assertFalse(self.object._needs_updating)
                self.assertFalse(self.object._needs_deleting)
                self.assertFalse(self.object._image_needs_setting)
                self.assertFalse(self.object._image_needs_deleting)

                self.assertIsNone(self.object.interop_id)

        # Successfully add, then test unsuccessful update and delete.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()

            # Login.
            self.client.wait_for_server()
            self.client.login()

            # Successfully add object to server.
            server.set_post_object_response(self.object_data, 1)
            self.object.sync()

            # Test unsuccessful update on server.
            # Update object.
            updated_object = self.object_data.copy()
            updated_object["shape"] = "circle"
            self.object.update(json.dumps(updated_object))

            # Try to update on server.
            codes = [400, 404, 500]
            for code in codes:
                server.set_put_object_response(1, self.object_data, code=code)
                self.object.sync()

                self.assertFalse(self.object._needs_adding)
                self.assertTrue(self.object._needs_updating)
                self.assertFalse(self.object._needs_deleting)
                self.assertFalse(self.object._image_needs_setting)
                self.assertFalse(self.object._image_needs_deleting)

            # Delete object.
            self.object.delete()

            # Test unsuccessful delete on server.
            # Try to delete object from server.
            for code in codes:
                server.set_delete_object_response(1, code=code)
                self.object.sync()

                self.assertFalse(self.object._needs_adding)
                self.assertFalse(self.object._needs_updating)
                self.assertTrue(self.object._needs_deleting)
                self.assertFalse(self.object._image_needs_setting)
                self.assertFalse(self.object._image_needs_deleting)

    def test_state_variables_after_unsuccessful_image_sync(self):
        """Tests that the state variables reflect unsuccessful object image
        syncs.
        """
        # Successfully add the object.
        with InteroperabilityMockServer("http://interop") as server:
            # Setup mock server.
            server.set_root_response()
            server.set_login_response()
            server.set_post_object_response(self.object_data, 1)

            self.client.wait_for_server()
            self.client.login()
            self.object.sync()

        # Test the unsuccessful setting of an image on the server.
        # Set an image.
        self.object.set_image(generate_image())

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
                server.set_post_object_image_response(1, code=code)
                self.object.sync()

                self.assertFalse(self.object._needs_adding)
                self.assertFalse(self.object._needs_updating)
                self.assertFalse(self.object._needs_deleting)
                self.assertTrue(self.object._image_needs_setting)
                self.assertFalse(self.object._image_needs_deleting)

                self.assertFalse(self.object.image_is_on_server)

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
            self.object.set_image(generate_image())
            server.set_post_object_image_response(1)
            self.object.sync()

            # Delete the image.
            self.object.delete_image()

            # Try to delete the image from the server.
            codes = [400, 404, 500]
            for code in codes:
                server.set_delete_object_image_response(1, code=code)
                self.object.sync()

                self.assertFalse(self.object._needs_adding)
                self.assertFalse(self.object._needs_updating)
                self.assertFalse(self.object._needs_deleting)
                self.assertFalse(self.object._image_needs_setting)
                self.assertTrue(self.object._image_needs_deleting)

                self.assertTrue(self.object.image_is_on_server)


if __name__ == "__main__":
    rospy.init_node("test_local_objects")
    rosunit.unitrun("test_local_objects", "test_object", TestObject)
