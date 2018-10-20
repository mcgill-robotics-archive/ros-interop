#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Interoperability Telemetry ROS Client."""

import sys
import rospy
import itertools
import message_filters
from mavros_msgs.msg import Altitude
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from requests.exceptions import ConnectionError, HTTPError, Timeout
from interop import InteroperabilityClient, OfflineInteroperabilityClient


def update_telemetry(navsat_msg, altitude_msg, pose_msg):
    """Telemetry subscription callback.

    Args:
        navsat_msg: sensor_msgs/NavSatFix message.
        altitude_msg: mavros_msgs/Altitude message.
        pose_msg: geometry_msgs/PoseStamped message in ENU.
    """
    try:
        client.post_telemetry(navsat_msg, altitude_msg, pose_msg)
    except (ConnectionError, Timeout) as e:
        rospy.logwarn(e)
        return
    except (ValueError, HTTPError) as e:
        rospy.logerr(e)
        return
    except Exception as e:
        rospy.logfatal(e)
        return


if __name__ == "__main__":
    # Initialize node.
    rospy.init_node("telemetry")

    # Get server connection information.
    offline = rospy.get_param("~offline")
    if offline:
        base_path = rospy.get_param("~base_path")
        client = OfflineInteroperabilityClient(base_path)
        rospy.logwarn("Running in OFFLINE mode")
    else:
        base_url = rospy.get_param("~base_url")
        timeout = rospy.get_param("~timeout")
        verify = rospy.get_param("~verify")
        client = InteroperabilityClient.from_env(base_url, timeout, verify)

    # Wait for server to be reachable, then login.
    client.wait_for_server()
    try:
        client.login()
    except Exception as e:
        rospy.logfatal(e)
        sys.exit(1)

    # Get ROS parameters for synchronization queue size and time delay.
    sync_queue = rospy.get_param("~sync_queue_size")
    sync_delay = rospy.get_param("~max_sync_delay")

    # Get ROS parameters for subscribed topic names.
    navsat_topic = rospy.get_param("~navsat_topic")
    altitude_topic = rospy.get_param("~altitude_topic")
    pose_topic = rospy.get_param("~pose_topic")

    # Setup synchronized subscribers.
    subscribers = [
        message_filters.Subscriber(navsat_topic, NavSatFix),
        message_filters.Subscriber(altitude_topic, Altitude),
        message_filters.Subscriber(pose_topic, PoseStamped),
    ]
    synchronizer = message_filters.ApproximateTimeSynchronizer(
        subscribers, sync_queue, sync_delay)
    synchronizer.registerCallback(update_telemetry)

    # Spin forever.
    rospy.spin()
