# -*- coding: utf-8 -*-

"""Asynchronous Interoperability HTTP Client."""

import urllib
import serializers
from tornado.escape import json_decode
from httpclient_session import Session
from tornado.httpclient import HTTPClient, HTTPRequest


class InteroperabilityClient(object):

    """InteroperabilityClient.

    Attributes:
        url: Base URL of the Interoperability server.
        session: AsyncHTTPClient session.
        timeout: Timeout in seconds for individual requests.
    """

    def __init__(self, url, timeout=1.0):
        """Initializes InteroperabilityClient.

        Note: You must authenticate() before using any other methods.

        Args:
            url: Interoperability server base URL (e.g. http://127.0.0.1:8080).
            timeout: Timeout in seconds for individual requests.
        """
        self.timeout = timeout
        self.url = url[:-1] if url.endswith('/') else url
        self.session = Session(HTTPClient)

    def authenticate(self, username, password):
        """Authenticates with the Interoperability server with given
        credentials.

        Args:
            username: Interoperability server username.
            password: Interoperability server password.

        Raises:
            HTTPError: On failure or timeout.
        """
        self._post("/api/login",
                   body={"username": username, "password": password})

    def _get(self, uri, **kwargs):
        """Sends GET request to Interoperability server at specified URI.

        Args:
            uri: Server URI to access.
            **kwargs: Arguments to HTTPRequest.

        Returns:
            HTTPResponse.

        Raises:
            HTTPError: On failure or timeout.
        """
        request = HTTPRequest(
            url=self.url + (uri if uri.startswith('/') else '/' + uri),
            method="GET",
            request_timeout=self.timeout,
            **kwargs)
        response = self.session.fetch(request)
        return response

    def _put(self, uri, body=None, **kwargs):
        """Sends PUT request to Interoperability server at specified URI.

        Args:
            uri: Server URI to access.
            body: Dictionary of request's body.
            **kwargs: Arguments to HTTPRequest.

        Returns:
            HTTPResponse.

        Raises:
            HTTPError: On failure or timeout.
        """
        if isinstance(body, dict):
            body = urllib.urlencode(body)
        request = HTTPRequest(
            url=self.url + (uri if uri.startswith('/') else '/' + uri),
            method="PUT",
            body=body,
            request_timeout=self.timeout,
            **kwargs)
        response = self.session.fetch(request)
        return response

    def _post(self, uri, body=None, **kwargs):
        """Sends POST request to Interoperability server at specified URI.

        Args:
            uri: Server URI to access.
            body: Dictionary of request's body.
            **kwargs: Arguments to HTTPRequest.

        Returns:
            HTTPResponse.

        Raises:
            HTTPError: On failure or timeout.
        """
        if isinstance(body, dict):
            body = urllib.urlencode(body)
        request = HTTPRequest(
            url=self.url + (uri if uri.startswith('/') else '/' + uri),
            method="POST",
            body=body,
            request_timeout=self.timeout,
            **kwargs)
        response = self.session.fetch(request)
        return response

    def _delete(self, uri, **kwargs):
        """Sends DELETE request to Interoperability server at specified URI.

        Args:
            uri: Server URI to access.
            **kwargs: Arguments to HTTPRequest.

        Returns:
            HTTPResponse.

        Raises:
            HTTPError: On failure or timeout.
        """
        request = HTTPRequest(
            url=self.url + (uri if uri.startswith('/') else '/' + uri),
            method="DELETE",
            request_timeout=self.timeout,
            **kwargs)
        response = self.session.fetch(request)
        return response

    def get_server_info(self):
        """Returns server information.

        Returns:
            (std_msgs/String, std_msgs/Time, std_msgs/Time) tuple.
            The first is the server message, the second is the message
            timestamp and the last is the server time.

        Raises:
            HTTPError: On failure or timeout.
        """
        response = self._get("/api/server_info")
        json = json_decode(response.body)
        return serializers.ServerInfoDeserializer.from_json(json)

    def get_obstacles(self):
        """Returns obstacles as Markers.

        Returns:
            Tuple of two visualization_msgs/MarkerArray, MarkerArray) tuple.
            The first is of moving obstacles, and the latter is of stationary
            obstacles.

        Raises:
            HTTPError: On failure or timeout.
        """
        response = self._get("/api/obstacles")
        json = json_decode(response.body)
        return serializers.ObstaclesDeserializer.from_json(json)

    def post_telemetry(self, navsat_msg, compass_msg):
        """Uploads telemetry information to Interoperability server.

        Args:
            navsat_msg: sensor_msgs/NavSatFix message.
            compass_msg: std_msgs/Float64 message in degrees.

        Returns:
            HTTPResponse.

        Raises:
            HTTPError: On failure or timeout.
        """
        json = serializers.TelemetrySerializer.from_msg(navsat_msg,
                                                        compass_msg)
        response = self._post("/api/telemetry", body=json)
        return response


if __name__ == "__main__":
    import rospy
    from visualization_msgs.msg import MarkerArray

    # Initialize node.
    rospy.init_node("test_interop_client")

    # Setup publishers.
    rate = rospy.Rate(24)
    moving = rospy.Publisher("/moving_obstacles", MarkerArray, queue_size=1)
    stationary = rospy.Publisher("/stationary_obstacles",
                                 MarkerArray, queue_size=1)

    # Initialize client.
    client = InteroperabilityClient("http://localhost:80")

    # Test out authentication and basic request.
    client.authenticate("testadmin", "testpass")
    print(client.get_server_info())

    # Publish obstacles.
    while not rospy.is_shutdown():
        moving_obj, stationary_obj = client.get_obstacles()
        moving.publish(moving_obj)
        stationary.publish(stationary_obj)
        rate.sleep()
