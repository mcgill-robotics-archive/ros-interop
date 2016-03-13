#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Interoperability Obstacles ROS Client."""

import rospy
from simplejson import JSONDecodeError
from interop import InteroperabilityClient
from visualization_msgs.msg import MarkerArray
from requests.exceptions import HTTPError, Timeout


def publish_obstacles(timer_event):
    """Requests and publishes obstacles.

    Args:
        timer_event: ROS TimerEvent.
    """
    try:
        moving_obstacles, stationary_obstacles = client.get_obstacles()
    except Timeout as e:
        rospy.logwarn(e)
        return
    except (JSONDecodeError, HTTPError) as e:
        rospy.logerr(e)
        return

    moving_pub.publish(moving_obstacles)
    stationary_pub.publish(stationary_obstacles)


if __name__ == "__main__":
    # Initialize node.
    rospy.init_node("obstacles")

    # Get ROS parameters for client.
    base_url = rospy.get_param("~base_url")
    username = rospy.get_param("~username")
    password = rospy.get_param("~password")
    timeout = rospy.get_param("~timeout", 1.0)

    # Initialize interoperability client.
    client = InteroperabilityClient(base_url, username, password, timeout)

    # Get ROS parameters for published topic names.
    moving_topic = rospy.get_param("~moving_topic", "~moving")
    stationary_topic = rospy.get_param("~stationary_topic", "~stationary")

    # Setup publishers.
    moving_pub = rospy.Publisher(moving_topic, MarkerArray, queue_size=1)
    stationary_pub = rospy.Publisher(stationary_topic,
                                     MarkerArray, queue_size=1)

    # Get ROS parameter for publishing period.
    # The default is a rate of 20Hz as a buffer.
    period = float(rospy.get_param("~period", 0.05))

    # Set up ROS timer for publishing at the specified rates.
    rospy.Timer(rospy.Duration(period), publish_obstacles)

    # Spin forever.
    rospy.spin()
