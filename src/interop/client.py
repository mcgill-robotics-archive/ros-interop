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
        """Initializes InteroperabilityClient and authenticates with server.

        Args:
            url: Interoperability server base URL (e.g. http://127.0.0.1:8080).
            username: Interoperability server username.
            password: Interoperability server password.
            timeout: Timeout in seconds for individual requests.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
        """
        self.timeout = timeout
        self.url = url[:-1] if url.endswith('/') else url
        self.session = requests.Session()

        self.__credentials = {"username": username, "password": password}

        # Login.
        self.__login()

    def __login(self):
        """Authenticates with the server."""
        response = self.session.request(
            method="POST",
            url=self.url + "/api/login",
            timeout=self.timeout,
            data=self.__credentials)
        response.raise_for_status()

    def __request(self, method, uri, **kwargs):
        """Sends request to Interoperability server at specified URI.

        Args:
            method: HTTP method.
            uri: Server URI to access.
            **kwargs: Arguments to HTTP Request.

        Returns:
            HTTP Response.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
        """
        # Try until authenticated.
        while True:
            # Send request.
            response = self.session.request(
                method=method,
                url=self.url + (uri if uri.startswith('/') else '/' + uri),
                timeout=self.timeout,
                **kwargs)

            # Relogin if session expired, and try again.
            if response.status_code == requests.codes.FORBIDDEN:
                # Start a new session.
                self.session.close()
                self.session = requests.Session()

                # Relogin.
                self.__login()
                continue

            # Notify of other errors.
            response.raise_for_status()

            # All is good.
            break

        return response

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
        return self.__request("GET", uri, **kwargs)

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
        return self.__request("PUT", uri, **kwargs)

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
        return self.__request("POST", uri, **kwargs)

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
        return self.__request("DELETE", uri, **kwargs)

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

    def get_obstacles(self, frame, lifetime):
        """Returns obstacles as Markers.

        Args:
            frame: Frame ID of every Marker.
            lifetime: Lifetime of every Marker in seconds.

        Returns:
            Tuple of two visualization_msgs/MarkerArray.
            The first is of moving obstacles, and the latter is of stationary
            obstacles.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            JSONDecodeError: On JSON decoding failure.
        """
        response = self._get("/api/obstacles")
        return serializers.ObstaclesDeserializer.from_json(response.json(),
                                                           frame, lifetime)

    def post_telemetry(self, navsat_msg, compass_msg):
        """Uploads telemetry information to Interoperability server.

        Args:
            navsat_msg: sensor_msgs/NavSatFix message.
            compass_msg: std_msgs/Float64 message in degrees.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
        """
        json = serializers.TelemetrySerializer.from_msg(navsat_msg,
                                                        compass_msg)
        self._post("/api/telemetry", data=json)

    def get_targets(self):
        """Returns first 100 submitted targets.

        Returns:
            Dictionary of Target IDs to Target ROS messages.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            JSONDecodeError: On JSON decoding failure.
        """
        response = self._get("/api/targets")
        targets = {
            t["id"]: serializers.TargetSerializer.from_json(t)
            for t in response.json()
        }
        return targets

    def post_target(self, target):
        """Uploads new target for submission.

        Args:
            target: Target ROS message.

        Returns:
            Target ID.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            JSONDecodeError: On JSON decoding failure.
        """
        json = serializers.TargetSerializer.from_msg(target)
        response = self._post("/api/targets", data=json)
        return response.json()["id"]

    def get_target(self, id):
        """Returns target with matching ID.

        Args:
            id: Target ID.

        Returns:
            A Target ROS message.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            JSONDecodeError: On JSON decoding failure.
        """
        response = self._get("/api/targets/" + int(id))
        target = serializers.TargetSerializer.from_json(response.json())
        return target

    def put_target(self, id, target):
        """Updates target information.

        Args:
            id: Target ID.
            target: Target ROS message.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
        """
        json = serializers.TargetSerializer.from_msg(target)
        self._put("/api/targets/" + int(id), data=json)

    def delete_target(self, id):
        """Deletes target with matching ID.

        Args:
            id: Target ID.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
        """
        self._delete("/api/targets/" + int(id))
