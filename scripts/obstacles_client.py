#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Interoperability Obstacles ROS Client."""

import sys
import rospy
from requests.exceptions import ConnectionError, HTTPError, Timeout
from interop.msg import GeoCylinderArrayStamped
from interop import InteroperabilityClient, OfflineInteroperabilityClient


def publish_obstacles(timer_event):
    """Requests and publishes obstacles.

    Args:
        timer_event: ROS TimerEvent.
    """
    try:
        stationary_obstacles = client.get_obstacles(frame, lifetime)
    except (ConnectionError, Timeout) as e:
        rospy.logwarn(e)
        return
    except (ValueError, HTTPError) as e:
        rospy.logerr(e)
        return
    except Exception as e:
        rospy.logfatal(e)
        return

    stationary_pub.publish(stationary_obstacles)


if __name__ == "__main__":
    # Initialize node.
    rospy.init_node("obstacles")

    # Get server connection information.
    offline = rospy.get_param("~offline")
    if offline:
        base_path = rospy.get_param("~base_path")
        no_moving_obstacles = rospy.get_param("~no_moving_obstacles")
        client = OfflineInteroperabilityClient(base_path)
        rospy.logwarn("Running in OFFLINE mode")
    else:
        base_url = rospy.get_param("~base_url")
        timeout = rospy.get_param("~timeout")
        verify = rospy.get_param("~verify")
        no_moving_obstacles = False
        client = InteroperabilityClient.from_env(base_url, timeout, verify)

    # Wait for server to be reachable, then login.
    client.wait_for_server()
    try:
        client.login()
    except Exception as e:
        rospy.logfatal(e)
        sys.exit(1)

    # Get ROS parameters for published topic names.
    stationary_topic = rospy.get_param("~stationary_topic")

    # Setup publishers.
    stationary_pub = rospy.Publisher(
        stationary_topic, GeoCylinderArrayStamped, queue_size=1)

    # Get ROS parameter for publishing period and frame ID.
    period = float(rospy.get_param("~period"))
    frame = str(rospy.get_param("~frame"))
    lifetime = 2 * period

    # Set up ROS timer for publishing at the specified rates.
    rospy.Timer(rospy.Duration(period), publish_obstacles)

    # Spin forever.
    rospy.spin()
