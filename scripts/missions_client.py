#!/usr/bin/env python
"""Mission Information Client"""

import sys
import rospy
from threading import Lock
from functools import partial
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from interop.srv import GetMissionByID
from geographic_msgs.msg import GeoPointStamped
from interop.msg import FlyZoneArray, WayPoints, GeoPolygonStamped
from requests.exceptions import Timeout, ConnectionError, HTTPError
from interop import InteroperabilityClient, OfflineInteroperabilityClient


def publish_mission(event):
    """Requests and publishes the mission information.

    Args:
        event (TimerEvent): Timer event (unused).
    """
    with lock:
        # Update the time stamp of all the messages manually since we reuse
        # cached messages, as we don't expect them to change.
        stamp = rospy.get_rostime()
        update_stamp(stamp, msgs)

        # Publish.
        flyzones_pub.publish(msgs[0])
        search_grid_pub.publish(msgs[1])
        waypoints_pub.publish(msgs[2])
        air_drop_pub.publish(msgs[3])
        off_axis_obj_pub.publish(msgs[4])
        emergent_obj_pub.publish(msgs[5])
        home_pub.publish(msgs[6])


def update_stamp(stamp, msg):
    """Recursively update the time stamp of all the messages.

    Args:
        stamp (Time): Time stamp to set all header stamps to.
        msg ([AnyMsg] or AnyMsg): Any ROS message or list of ROS messages.
    """
    # If given a list, recursively update each element separately.
    if hasattr(msg, "__iter__"):
        map(partial(update_stamp, stamp), msg)
        return

    # If a header was found, update the time stamp.
    if isinstance(msg, Header):
        msg.stamp = stamp
        return

    # Recursively update all message fields.
    if hasattr(msg, "__slots__"):
        map(lambda m: update_stamp(stamp, getattr(msg, m)), msg.__slots__)


def get_active_mission(req):
    """ Service to update mission information with current active mission.

    Args:
        req: Request of type Trigger.

    Returns:
        TriggerResponse with true, false for success, failure.
    """
    with lock:
        global msgs
        try:
            msgs = client.get_active_mission(frame)
        except (ConnectionError, Timeout) as e:
            rospy.logwarn(e)
            return False, str(e)
        except (ValueError, HTTPError) as e:
            rospy.logerr(e)
            return False, str(e)

    rospy.loginfo("Using active mission")
    return True, "Success"


def get_mission_by_id(req):
    """ Service to update mission information with specific mission as given
    by id.

    Args:
        req: GetMissionByID type request with field id corresponding to the
             mission

    Returns:
        GetMissionByIdResponse which is true for success and false for
        failure.
    """
    with lock:
        global msgs
        try:
            msgs = client.get_mission(req.id, frame)
        except (ConnectionError, Timeout) as e:
            rospy.logwarn(e)
            return False, str(e)
        except (ValueError, HTTPError) as e:
            rospy.logerr(e)
            return False, str(e)
        except Exception as e:
            rospy.logfatal(e)
            return False, str(e)

    rospy.loginfo("Using mission ID: %d", req.id)
    return True, "Success"


if __name__ == "__main__":
    rospy.init_node("mission_info")

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

    # Login.
    client.wait_for_server()
    try:
        client.login()
    except Exception as e:
        rospy.logfatal(e)
        sys.exit(1)

    # Get topics to publish to.
    flyzones_topic = rospy.get_param("~flyzones_topic")
    search_grid_topic = rospy.get_param("~search_grid_topic")
    waypoints_topic = rospy.get_param("~waypoints_topic")
    air_drop_topic = rospy.get_param("~air_drop_topic")
    off_axis_obj_topic = rospy.get_param("~off_axis_obj_topic")
    emergent_obj_topic = rospy.get_param("~emergent_obj_topic")
    home_topic = rospy.get_param("~home_topic")

    # Setup publishers.
    flyzones_pub = rospy.Publisher(flyzones_topic, FlyZoneArray, queue_size=1)
    search_grid_pub = rospy.Publisher(
        search_grid_topic, GeoPolygonStamped, queue_size=1)
    waypoints_pub = rospy.Publisher(waypoints_topic, WayPoints, queue_size=1)
    air_drop_pub = rospy.Publisher(
        air_drop_topic, GeoPointStamped, queue_size=1)
    off_axis_obj_pub = rospy.Publisher(
        off_axis_obj_topic, GeoPointStamped, queue_size=1)
    emergent_obj_pub = rospy.Publisher(
        emergent_obj_topic, GeoPointStamped, queue_size=1)
    home_pub = rospy.Publisher(home_topic, GeoPointStamped, queue_size=1)

    # Get message parameters.
    frame = str(rospy.get_param("~frame"))

    # Create timer event and publish.
    period = float(rospy.get_param("~period"))

    # Create lock for msgs.
    lock = Lock()

    # Setup services and publish mission.
    rospy.Service("get_active_mission", Trigger, get_active_mission)
    rospy.Service("get_mission_by_id", GetMissionByID, get_mission_by_id)

    # Get mission to begin publishing. This is the first mission published.
    mission_id = rospy.get_param("~id")
    msgs = None

    retry_rate = rospy.Rate(1)
    while msgs is None and not rospy.is_shutdown():
        # If id is negative then it is default and non existent.
        if mission_id >= 0:
            req = GetMissionByID()
            req.id = mission_id
            get_mission_by_id(req)
        else:
            get_active_mission(Trigger())

        retry_rate.sleep()

    # Publish message on timer.
    timer = rospy.Timer(rospy.Duration(period), publish_mission)

    rospy.spin()
