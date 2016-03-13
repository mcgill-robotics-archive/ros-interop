# -*- coding: utf-8 -*-

"""Interoperability HTTP Client."""

import requests
import serializers


class InteroperabilityClient(object):

    """InteroperabilityClient.

    Attributes:
        url: Base URL of the Interoperability server.
        session: Requests session.
        timeout: Timeout in seconds for individual requests.
    """

    def __init__(self, url, username, password, timeout):
        """Initializes InteroperabilityClient.

        Args:
            url: Interoperability server base URL (e.g. http://127.0.0.1:8080).
            username: Interoperability server username.
            password: Interoperability server password.
            timeout: Timeout in seconds for individual requests.
        """
        self.timeout = timeout
        self.url = url[:-1] if url.endswith('/') else url
        self.session = requests.Session()

        self._post("/api/login",
                   data={"username": username, "password": password})

    def _get(self, uri, **kwargs):
        """Sends GET request to Interoperability server at specified URI.

        Args:
            uri: Server URI to access.
            **kwargs: Arguments to HTTP Request.

        Returns:
            HTTP Response.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
        """
        response = self.session.request(
            method="GET",
            url=self.url + (uri if uri.startswith('/') else '/' + uri),
            timeout=self.timeout,
            **kwargs)
        response.raise_for_status()
        return response

    def _put(self, uri, **kwargs):
        """Sends PUT request to Interoperability server at specified URI.

        Args:
            uri: Server URI to access.
            **kwargs: Arguments to HTTP Request.

        Returns:
            HTTP Response.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
        """
        response = self.session.request(
            method="PUT",
            url=self.url + (uri if uri.startswith('/') else '/' + uri),
            timeout=self.timeout,
            **kwargs)
        response.raise_for_status()
        return response

    def _post(self, uri, **kwargs):
        """Sends POST request to Interoperability server at specified URI.

        Args:
            uri: Server URI to access.
            **kwargs: Arguments to HTTP Request.

        Returns:
            HTTP Response.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
        """
        response = self.session.request(
            method="POST",
            url=self.url + (uri if uri.startswith('/') else '/' + uri),
            **kwargs)
        response.raise_for_status()
        return response

    def _delete(self, uri, **kwargs):
        """Sends DELETE request to Interoperability server at specified URI.

        Args:
            uri: Server URI to access.
            **kwargs: Arguments to HTTP Request.

        Returns:
            HTTP Response.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
        """
        response = self.session.request(
            method="DELETE",
            url=self.url + (uri if uri.startswith('/') else '/' + uri),
            timeout=self.timeout,
            **kwargs)
        response.raise_for_status()
        return response

    def get_server_info(self):
        """Returns server information.

        Returns:
            (std_msgs/String, std_msgs/Time, std_msgs/Time) tuple.
            The first is the server message, the second is the message
            timestamp and the last is the server time.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            JSONDecodeError: On JSON decoding failure.
        """
        response = self._get("/api/server_info")
        return serializers.ServerInfoDeserializer.from_json(response.json())

    def get_obstacles(self, lifetime):
        """Returns obstacles as Markers.

        Args:
            lifetime: Lifetime of every Marker in seconds.

        Returns:
            Tuple of two visualization_msgs/MarkerArray tuple.
            The first is of moving obstacles, and the latter is of stationary
            obstacles.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            JSONDecodeError: On JSON decoding failure.
        """
        response = self._get("/api/obstacles")
        return serializers.ObstaclesDeserializer.from_json(response.json(),
                                                           lifetime)

    def post_telemetry(self, navsat_msg, compass_msg):
        """Uploads telemetry information to Interoperability server.

        Args:
            navsat_msg: sensor_msgs/NavSatFix message.
            compass_msg: std_msgs/Float64 message in degrees.

        Returns:
            HTTP Response.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            JSONDecodeError: On JSON decoding failure.
        """
        json = serializers.TelemetrySerializer.from_msg(navsat_msg,
                                                        compass_msg)
        response = self._post("/api/telemetry", data=json)
        return response
