# -*- coding: utf-8 -*-
"""Interoperability HTTP Client."""

import os
import json
import rospy
import requests
import serializers


class InteroperabilityClient(object):

    """InteroperabilityClient.

    Attributes:
        url: Base URL of the Interoperability server.
        session: Requests session.
        timeout: Timeout in seconds for individual requests.
    """

    def __init__(self, url, username, password, timeout=1.0, verify=True):
        """Initializes InteroperabilityClient.

        Note: the client must wait_for_server() and login() to the server
        before first use.

        Args:
            url: Interoperability server base URL (e.g. http://127.0.0.1:8080).
            username: Interoperability server username.
            password: Interoperability server password.
            timeout: Timeout in seconds for individual requests, default: 1.0s.
            verify: Whether to verify SSL certificates or not, default: True.
        """
        if not url.strip():
            raise ValueError("Base URL cannot be empty")

        self.verify = verify
        self.timeout = timeout
        self.url = url[:-1] if url.endswith('/') else url
        self.session = requests.Session()

        # Set up credentials for login.
        self.__credentials = {"username": username, "password": password}

    @classmethod
    def from_env(cls, url, *args, **kwargs):
        """Initializes an InteroperabilityClient with credentials loaded from
        environment variables.

        Args:
            url: Interoperability server base URL (e.g. http://127.0.0.1:8080).

        The username must be stored in $INTEROP_USERNAME.
        The password must be stored in $INTEROP_PASSWORD.

        Args:
            *args: Additional positional arguments to pass to the constructor.
            **kwargs: Additional key-word arguments to pass to the constructor.

        Returns:
            An InteroperabilityClient.
        """
        username = os.environ["INTEROP_USERNAME"]
        password = os.environ["INTEROP_PASSWORD"]

        return InteroperabilityClient(url, username, password, *args, **kwargs)

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
            ConnectionError: On connection failure.
        """
        # Try until authenticated.
        response = requests.Response()
        while not rospy.is_shutdown():
            # Send request.
            response = self.session.request(
                method=method,
                url=self.url + (uri if uri.startswith('/') else '/' + uri),
                timeout=self.timeout,
                verify=self.verify,
                **kwargs)

            # Relogin if session expired, and try again.
            if response.status_code == requests.codes.FORBIDDEN:
                rospy.logwarn("Session expired: reauthenticating...")

                # Start a new session.
                self.session.close()
                self.session = requests.Session()

                # Relogin.
                self.login()
                continue

            message = self._get_response_log_message(method, uri, response)

            # Notify of other errors.
            try:
                response.raise_for_status()
                rospy.logdebug(message)
            except requests.HTTPError:
                # For more human-readable error messages.
                raise requests.HTTPError(message, response=response)

            # All is good.
            break

        return response

    def _get_response_log_message(self, method, uri, response):
        """Constructs a user-friendly log message.

        Args:
            method: Request method.
            uri: Request URI.
            response: Response received.

        Returns:
            Log message.
        """
        return "[{status_code}] {method} {uri}: {reason} {content}".format(
            status_code=response.status_code,
            method=method,
            uri=uri,
            reason=response.reason,
            content=response.content)

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
            ConnectionError: On connection failure.
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
            ConnectionError: On connection failure.
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
            ConnectionError: On connection failure.
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
            ConnectionError: On connection failure.
        """
        return self.__request("DELETE", uri, **kwargs)

    def wait_for_server(self):
        """Waits until interoperability server is reachable."""
        reachable = False
        while not reachable and not rospy.is_shutdown():
            try:
                response = requests.get(
                    self.url, timeout=self.timeout, verify=self.verify)
                response.raise_for_status()
                reachable = response.ok
            except:
                continue

    def login(self):
        """Authenticates with the server.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        method = "POST"
        uri = "/api/login"
        response = self.session.request(
            method=method,
            url=self.url + uri,
            timeout=self.timeout,
            verify=self.verify,
            data=self.__credentials)
        message = self._get_response_log_message(method, uri, response)

        try:
            response.raise_for_status()
            rospy.logdebug(message)
        except requests.HTTPError:
            # For more human-readable error messages.
            raise requests.HTTPError(message, response=response)

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
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
        """
        response = self._get("/api/obstacles")
        return serializers.ObstaclesDeserializer.from_dict(
            response.json(), frame, lifetime)

    def post_telemetry(self, navsat_msg, compass_msg):
        """Uploads telemetry information to Interoperability server.

        Args:
            navsat_msg: sensor_msgs/NavSatFix message.
            compass_msg: std_msgs/Float64 message in degrees.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        dict_telem = serializers.TelemetrySerializer.from_msg(
            navsat_msg, compass_msg)
        self._post("/api/telemetry", data=dict_telem)

    def post_target(self, json_target):
        """Uploads new target for submission.

        Args:
            json_target: Target as a JSON string.

        Returns:
            Target ID.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        response = self._post("/api/odlcs", data=json_target)
        return response.json()["id"]

    def get_all_targets(self):
        """Returns first 100 submitted targets.

        Returns:
            dict: Target IDs to target data (dict).

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
        """
        response = self._get("/api/odlcs")
        targets = {t["id"]: t for t in response.json()}
        return targets

    def get_target(self, id):
        """Returns target with matching ID.

        Args:
            id: Target ID.

        Returns:
            dict: The target data.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
        """
        response = self._get("/api/odlcs/{:d}".format(id))
        return response.json()

    def get_active_mission(self, frame):
        """Gets active mission.

        Args:
            frame: Frame ID.

        Returns
            A tuple of (FlyZoneArray, GeoPolygonStamped, GeoObjectArray,
            GeoPointStamped, GeoPointStamped, GeoPointStamped, GeoPointStamped)
            corresponding to the flyzones, search grid, waypoints,
            air drop position, off axis target location, the emergent object
            location, the home position

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
            LookupError: On no active missions found.
        """
        response = self._get("/api/missions")
        for m in response.json():
            if m["active"]:
                return serializers.MissionDeserializer.from_dict(m, frame)
        raise LookupError("No active missions found")

    def get_all_missions(self, frame):
        """Gets all missions.

        Args:
            frame: Frame ID.

        Returns:
            A tuple of (FlyZoneArray, GeoPolygonStamped, GeoObjectArray,
            GeoPointStamped, GeoPointStamped, GeoPointStamped, GeoPointStamped)
            corresponding to the flyzones, search grid, waypoints,
            air drop position, off axis target location, the emergent object
            location, the home position.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
        """
        response = self._get("/api/missions")
        missions = {
            m["id"]: serializers.MissionDeserializer.from_dict(m, frame)
            for m in response.json()
        }
        return missions

    def get_mission(self, id, frame):
        """Returns mission with the matching ID.

        Args:
            id: Mission ID.
            frame: Frame ID.

        Returns:
            A tuple of (FlyZoneArray, GeoPolygonStamped, WayPoints,
            GeoPointStamped, GeoPointStamped, GeoPointStamped, GeoPointStamped)
            corresponding to the flyzones, search grid, waypoints, air drop
            position, off axis target location, the emergent object location,
            and the home position.

        Raises:
            Timeout: On Timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
        """
        response = self._get("/api/missions/{:d}".format(id))
        mission = serializers.MissionDeserializer.from_dict(
            response.json(), frame)
        return mission

    def put_target(self, id, json_target):
        """Updates target information.

        Args:
            id: Target ID.
            json_target: Target as a JSON string.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        self._put("/api/odlcs/{:d}".format(id), data=json_target)

    def delete_target(self, id):
        """Deletes target with matching ID.

        Args:
            id: Target ID.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        self._delete("/api/odlcs/{:d}".format(id))

    def post_target_image(self, id, png):
        """Adds or updates target image thumbnail as a compressed PNG.

        Args:
            id: Target ID.
            png: Target PNG image from a file.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            CvBridgeError: On image conversion failure.
        """
        self._post("/api/odlcs/{:d}/image".format(id), data=png)

    def get_target_image(self, id):
        """Retrieves target image thumbnail.

        Args:
            id: Target ID.

        Returns:
            A ROS Image message.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            CvBridgeError: On image conversion failure.
        """
        response = self._get("/api/odlcs/{:d}/image".format(id))
        img = serializers.TargetImageSerializer.from_raw(response.content)
        return img

    def delete_target_image(self, id):
        """Deletes target image thumbnail.

        Args:
            id: Target ID.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        self._delete("/api/odlcs/{:d}/image".format(id))
