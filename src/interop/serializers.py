# -*- coding: utf-8 -*-

"""Interoperability API message serializer."""

import cv2
import utm
import rospy
import numpy as np
import dateutil.parser
from dateutil.tz import tzutc
from datetime import datetime
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header, String, Time
from interop.msg import Color, Orientation, Shape, Target, TargetType


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
    def from_json(cls, json, frame, lifetime):
        """Deserializes obstacle data into two MarkerArrays.

        Args:
            json: JSON dictionary.
            frame: Frame ID of every Marker.
            lifetime: Lifetime of every Marker in seconds.

        Returns:
            Tuple of two visualization_msgs/MarkerArray, MarkerArray) tuple.
            The first is of moving obstacles, and the latter is of stationary
            obstacles.
        """
        # Generate base header.
        header = Header()
        header.stamp = rospy.get_rostime()
        header.frame_id = frame

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

    # Enumeration message types.
    ENUMERATION_TYPES = {
        Color,
        Orientation,
        Shape,
        TargetType
    }

    @classmethod
    def from_msg(cls, msg):
        """Serializes target data into a dictionary.

        Args:
            msg: Target ROS message.

        Returns:
            JSON dictionary.
        """
        json = {}

        # Set all fields.
        for attribute in msg.__slots__:
            value = getattr(msg, attribute)
            if type(value) in cls.ENUMERATION_TYPES:
                value = value.data

            json[attribute] = value

        return json

    @classmethod
    def from_json(cls, json):
        """Deserializes target data into Target ROS message.

        Args:
            json: JSON dictionary.

        Returns:
            A Target ROS message.
        """
        # Determine ROS message type.
        msg = Target()

        for attribute in msg.__slots__:
            if attribute in json:
                # Get corresponding type of slot.
                attribute_type = type(getattr(msg, attribute))

                # Set 'casted' value.
                setattr(msg, attribute_type(json[attribute]))

        return msg


class TargetImageSerializer(object):

    """Target image message serializer."""

    @classmethod
    def from_msg(cls, msg):
        """Serializes a ROS Image message into a compressed PNG image.

        Args:
            msg: ROS Image message.

        Returns:
            Compressed PNG image.

        Raises:
            CvBridgeError: On image conversion error.
        """
        # Convert ROS Image to OpenCV image.
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg)

        # Convert to PNG with highest level of compression to limit bandwidth
        # usage. PNG is used since it is a lossless format, so this can later
        # be retrieved as a ROS image without issue.
        compression = [cv2.IMWRITE_PNG_COMPRESSION, 9]
        png = cv2.imencode(".png", img, compression)[1].tostring()

        return png

    @classmethod
    def from_raw(cls, raw):
        """Deserializes binary-encoded image data into a ROS Image message.

        Args:
            raw: Binary encoded image data.

        Returns:
            ROS Image message.

        Raises:
            CvBridgeError: On image conversion error.
        """
        # Convert to OpenCV image.
        nparr = np.fromstring(raw, np.uint8)
        img = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_COLOR)

        # Convert to ROS message.
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(img)

        return msg
