#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Interoperability Telemetry ROS Client."""

import rospy
import itertools
import message_filters
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from simplejson import JSONDecodeError
from interop import InteroperabilityClient
from requests.exceptions import HTTPError, Timeout


def update_telemetry(navsat_msg, compass_msg):
    """Telemetry subscription callback.

    Args:
        navsat_msg: sensor_msgs/NavSatFix message.
        compass_msg: std_msgs/Float64 message in degrees.
    """
    try:
        client.post_telemetry(navsat_msg, compass_msg)
    except Timeout as e:
        rospy.logwarn(e)
        return
    except (JSONDecodeError, HTTPError) as e:
        rospy.logerr(e)
        return


class UnstampedTimeSynchronizer(message_filters.ApproximateTimeSynchronizer):

    """Synchronizes messages by order of arrival.

    This is to allow synchronization between a mixture of stamped and
    unstamped messages. Only to be used when the rate is approximately the
    same, and the queue size is small.
    """

    def add(self, msg, my_queue):
        """Adds a message to the current queue, and matches them accordingly.

        Args:
            msg: Message.
            my_queue: Current message queue map from ROS Time to ROS message.
        """
        # Store when this message was received.
        received = rospy.get_rostime()

        # Acquire lock.
        self.lock.acquire()

        # Add to queue.
        my_queue[received] = msg
        while len(my_queue) > self.queue_size:
            del my_queue[min(my_queue)]

        # Signal by approximate time, as per ros_comm source code.
        # Credits to Willow Garage, Inc.
        # TODO: Clean this up.
        for vv in itertools.product(*[list(q.keys()) for q in self.queues]):
            qt = zip(self.queues, vv)
            if (((max(vv) - min(vv)) < self.slop) and
                    (len([1 for q, t in qt if t not in q]) == 0)):
                msgs = [q[t] for q, t in qt]
                self.signalMessage(*msgs)
                for q, t in qt:
                    del q[t]

        # Unlock.
        self.lock.release()


if __name__ == "__main__":
    # Initialize node.
    rospy.init_node("interop")

    # Get ROS parameters for client.
    base_url = rospy.get_param("~base_url")
    username = rospy.get_param("~username")
    password = rospy.get_param("~password")
    timeout = rospy.get_param("~timeout", 1.0)

    # Initialize interoperability client.
    client = InteroperabilityClient(base_url, username, password, timeout)

    # Get ROS parameters for synchronization queue size and time delay.
    sync_queue = rospy.get_param("~sync_queue_size", 2)
    sync_delay = rospy.get_param("~max_sync_delay", 1)

    # Get ROS parameters for subscribed topic names.
    navsat_topic = rospy.get_param("~navsat_topic")
    compass_topic = rospy.get_param("~compass_topic")

    # Setup synchronized subscribers.
    subscribers = [
        message_filters.Subscriber(navsat_topic, NavSatFix),
        message_filters.Subscriber(compass_topic, Float64)
    ]
    synchronizer = UnstampedTimeSynchronizer(
        subscribers,
        sync_queue,
        sync_delay)
    synchronizer.registerCallback(update_telemetry)

    # Spin forever.
    rospy.spin()
