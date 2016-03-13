#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Interoperability Server Information ROS Client."""

import rospy
from std_msgs.msg import String, Time
from interop import InteroperabilityClient


def publish_server_info(timer_event):
    """Requests and publishes server information."""
    message, message_timestamp, server_time = client.get_server_info()
    msg_pub.publish(message)
    msg_timestamp_pub.publish(message_timestamp)
    time_pub.publish(server_time)


if __name__ == "__main__":
    # Initialize node.
    rospy.init_node("server_info")

    # Get ROS parameters for client.
    base_url = rospy.get_param("~base_url")
    username = rospy.get_param("~username")
    password = rospy.get_param("~password")

    # Initialize interoperability client.
    client = InteroperabilityClient(base_url, username, password)

    # Get ROS parameters for published topic names.
    msg_topic = rospy.get_param("~msg_topic", "~message")
    timestamp_topic = rospy.get_param("~timestamp_topic", "~message_timestamp")
    time_topic = rospy.get_param("~server_time_topic", "~time")

    # Setup publishers.
    msg_pub = rospy.Publisher(msg_topic, String, queue_size=1)
    msg_timestamp_pub = rospy.Publisher(timestamp_topic, Time, queue_size=1)
    time_pub = rospy.Publisher(time_topic, Time, queue_size=1)

    # Get ROS parameter for publishing periods.
    # The default is a rate of 20Hz.
    period = float(rospy.get_param("~period", 0.05))

    # Set up ROS timers for publishing at the specified rates.
    rospy.Timer(rospy.Duration(period), publish_server_info)

    # Spin forever.
    rospy.spin()
