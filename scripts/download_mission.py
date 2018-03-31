#!/usr/bin/env python
"""This script downloads all mission information for offline use."""

import os
import sys
import rospy
from interop import InteroperabilityClient

if __name__ == "__main__":
    rospy.init_node("mission_downloader")

    # Get target download path.
    if "INTEROP_PATH" in os.environ:
        base_path = rospy.get_param("~base_path", os.environ["INTEROP_PATH"])
    else:
        base_path = rospy.get_param("~base_path")

    # Get server login information.
    timeout = rospy.get_param("~timeout", 1.0)
    verify = rospy.get_param("~verify", True)
    if "INTEROP_HOST" in os.environ:
        base_url = rospy.get_param("~base_url", os.environ["INTEROP_HOST"])
    else:
        base_url = rospy.get_param("~base_url")

    # Initialize interoperability client.
    client = InteroperabilityClient.from_env(base_url, timeout, verify)

    # Login.
    client.wait_for_server()
    try:
        client.login()
    except Exception as e:
        rospy.logfatal(e)
        sys.exit(1)

    # Download.
    if not os.path.isdir(base_path):
        os.makedirs(base_path)
    client.download_mission_info(base_path)
    rospy.loginfo("Mission downloaded successfully to {}".format(base_path))
