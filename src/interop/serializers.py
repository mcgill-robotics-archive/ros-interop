# -*- coding: utf-8 -*-

"""Interoperability API message serializer."""

import utm
import rospy
import dateutil.parser
from dateutil.tz import tzutc
from datetime import datetime
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header, String, Time


def meters_to_feet(m):
    """Converts a distance from meters to feet.

    Args:
        m: Distance in meters.

    Returns:
        Distance in feet.
    """
    return float(m) / 0.3048


def feet_to_meters(ft):
    """Converts a distance from feet to meters.

    Args:
        ft: Distance in feet.

    Returns:
        Distance in meters.
    """
    return float(ft) * 0.3048


def iso8601_to_rostime(iso):
    """Converts ISO 8601 time to ROS Time.

    Args:
        iso: ISO 8601 encoded string.

    Returns:
        std_msgs/Time.
    """
    # Convert to datetime in UTC.
    t = dateutil.parser.parse(iso)
    if not t.utcoffset():
        t = t.replace(tzinfo=tzutc())

    # Convert to time from epoch in UTC.
    epoch = datetime.utcfromtimestamp(0)
    epoch = epoch.replace(tzinfo=tzutc())
    dt = t - epoch

    # Create ROS message.
    time = Time()
    time.data.secs = int(dt.total_seconds())
    time.data.nsecs = dt.microseconds * 1000

    return time


class ServerInfoDeserializer(object):

    """Server information deserializer."""

    @classmethod
    def from_json(cls, json):
        """Deserializes server information into a tuple of standard ROS
        messages.

        Args:
            json: JSON dictionary.

        Returns:
            (std_msgs/String, std_msgs/Time, std_msgs/Time) tuple.
            The first is the server message, the second is the message
            timestamp and the last is the server time.
        """
        message = String(data=json["message"])
        message_timestamp = iso8601_to_rostime(json["message_timestamp"])
        server_time = iso8601_to_rostime(json["server_time"])

        return (message, message_timestamp, server_time)


class ObstaclesDeserializer(object):

    """Obstacles message deserializer."""

    # McGill Robotics red, because we have big egos.
    OBSTACLE_COLOR = ColorRGBA(
        r=218 / 255.0,
        g=41 / 255.0,
        b=28 / 255.0,
        a=1.0)

    @classmethod
    def from_json(cls, json, lifetime):
        """Deserializes obstacle data into two MarkerArrays.

        Args:
            json: JSON dictionary.
            lifetime: Lifetime of every Marker in seconds.

        Returns:
            Tuple of two visualization_msgs/MarkerArray, MarkerArray) tuple.
            The first is of moving obstacles, and the latter is of stationary
            obstacles.
        """
        # Generate base header.
        header = Header()
        header.stamp = rospy.get_rostime()
        header.frame_id = "odom"

        # Parse moving obstacles, and populate markers with spheres.
        moving_obstacles = MarkerArray()
        if "moving_obstacles" in json:
            for obj in json["moving_obstacles"]:
                # Moving obstacles are spheres.
                marker = Marker()
                marker.header = header
                marker.type = marker.SPHERE
                marker.color = cls.OBSTACLE_COLOR
                marker.ns = "stationary_obstacles"
                marker.lifetime = rospy.Duration(lifetime)

                # Set scale as radius.
                radius = feet_to_meters(obj["sphere_radius"])
                marker.scale.x = marker.scale.y = marker.scale.z = radius

                # Convert latitude and longitude to UTM.
                easting, northing, _, _ = utm.from_latlon(obj["latitude"],
                                                          obj["longitude"])
                marker.pose.position.x = easting
                marker.pose.position.y = northing
                marker.pose.position.z = feet_to_meters(obj["altitude_msl"])

                moving_obstacles.markers.append(marker)

        # Parse stationary obstacles, and populate markers with cylinders.
        stationary_obstacles = MarkerArray()
        if "stationary_obstacles" in json:
            for obj in json["stationary_obstacles"]:
                # Stationary obstacles are cylinders.
                marker = Marker()
                marker.header = header
                marker.type = marker.CYLINDER
                marker.color = cls.OBSTACLE_COLOR
                marker.ns = "stationary_obstacles"
                marker.lifetime = rospy.Duration(lifetime)

                # Set scale to define size.
                radius = feet_to_meters(obj["cylinder_radius"])
                height = feet_to_meters(obj["cylinder_height"])
                marker.scale.x = marker.scale.y = radius
                marker.scale.z = height

                # Convert latitude and longitude to UTM.
                easting, northing, _, _ = utm.from_latlon(obj["latitude"],
                                                          obj["longitude"])
                marker.pose.position.x = easting
                marker.pose.position.y = northing
                marker.pose.position.z = height / 2

                stationary_obstacles.markers.append(marker)

        return (moving_obstacles, stationary_obstacles)


class TelemetrySerializer(object):

    """Telemetry message serializer."""

    @classmethod
    def from_msg(cls, navsat_msg, compass_msg):
        """Serializes telemetry data into a dictionary.

        Args:
            navsat_msg: sensor_msgs/NavSatFix message.
            compass_msg: std_msgs/Float64 message in degrees.

        Returns:
            JSON dictionary.
        """
        return {
            "latitude": float(navsat_msg.latitude),
            "longitude": float(navsat_msg.longitude),
            "altitude_msl": meters_to_feet(navsat_msg.altitude),
            "uas_heading": float(compass_msg.data)
        }


class TargetSerializer(object):

    """Target message serializer."""

    @classmethod
    def from_msg(cls, msg):
        """Serializes target data into a dictionary.

        Args:
            msg: Some ROS message (TBD).

        Returns:
            JSON dictionary.
        """
        raise NotImplementedError()

    @classmethod
    def from_json(cls, json):
        """Deserializes target data into relevant ROS message.

        Args:
            json: JSON dictionary.

        Returns:
            Some ROS message (TBD).
        """
        raise NotImplementedError()
