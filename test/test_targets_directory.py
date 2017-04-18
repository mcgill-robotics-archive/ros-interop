#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Test interop.local_targets.TargetsDirectory."""

import unittest
import shutil
import os
import os.path
import rospy
import rosunit
from interop.client import InteroperabilityClient
from interop.local_targets import TargetsDirectory


class TestTargetsDirectory(unittest.TestCase):

    """Tests the directory containing timestamped directories that will contain
    targets and their images.
    """

    def setUp(self):
        """Creates a directory used to store timestamped directories that will
        contain targets and their images.
        """
        # Set up the client.
        self.client = InteroperabilityClient("http://interop",  "testuser",
            "testpass", 1.0)

        # Set up the targets directory.
        self.targets_root = "/tmp/targets_root"
        self.targets_directory = TargetsDirectory(self.targets_root, self.client)

    def tearDown(self):
        """Cleans up after each test. Removes the targets root directory."""
        shutil.rmtree(self.targets_root)

    def test_that_a_symlink_and_directory_is_created(self):
        """Test that a symlink to the directory is created and that the symlink
        links to a directory.
        """
        # Test that the "latest" symlink is created.
        path_to_symlink = os.path.join(self.targets_root, "latest")
        self.assertTrue(os.path.exists(path_to_symlink))
        self.assertTrue(os.path.islink(path_to_symlink))

        # Extract the path pointed to by the symlink.
        path_pointed_by_symlink = os.readlink(path_to_symlink)  # May be relative.
        # Make potentially relative path into an absolute path.
        directory_path = os.path.join(os.path.dirname(path_to_symlink),
            path_pointed_by_symlink)

        # Test that the path pointed to by the symlink is a directory.
        self.assertTrue(os.path.isdir(directory_path))


if __name__ == "__main__":
    rospy.init_node("test_local_targets")
    rosunit.unitrun("test_local_targets", "test_targets_directory", TestTargetsDirectory)
