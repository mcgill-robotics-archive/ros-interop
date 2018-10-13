# -*- coding: utf-8 -*-
"""Interoperability HTTP Client."""

import os
import abc
import six
import json
import rospy
import requests
import serializers


@six.add_metaclass(abc.ABCMeta)
class BaseClient:

    """BaseClient."""

    @abc.abstractmethod
    def wait_for_server(self):
        """Waits until interoperability server is reachable."""
        raise NotImplementedError

    @abc.abstractmethod
    def login(self):
        """Authenticates with the server.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def get_obstacles(self, frame, lifetime):
        """Returns obstacles as Markers.

        Args:
            frame: Frame ID of every Marker.
            lifetime: Lifetime of every Marker in seconds.

        Returns:
            GeoCylinderArrayStamped of stationary obstacles.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
        """
        raise NotImplementedError

    @abc.abstractmethod
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
        raise NotImplementedError

    @abc.abstractmethod
    def get_active_mission(self, frame):
        """Gets active mission.

        Args:
            frame: Frame ID.

        Returns
            A tuple of (FlyZoneArray, GeoPolygonStamped, GeoObjectArray,
            GeoPointStamped, GeoPointStamped, GeoPointStamped, GeoPointStamped)
            corresponding to the flyzones, search grid, waypoints,
            air drop position, off axis object location, the emergent object
            location, the home position

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
            LookupError: On no active missions found.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def get_all_missions(self, frame):
        """Gets all missions.

        Args:
            frame: Frame ID.

        Returns:
            A tuple of (FlyZoneArray, GeoPolygonStamped, GeoObjectArray,
            GeoPointStamped, GeoPointStamped, GeoPointStamped, GeoPointStamped)
            corresponding to the flyzones, search grid, waypoints,
            air drop position, off axis object location, the emergent object
            location, the home position.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def get_mission(self, id, frame):
        """Returns mission with the matching ID.

        Args:
            id: Mission ID.
            frame: Frame ID.

        Returns:
            A tuple of (FlyZoneArray, GeoPolygonStamped, WayPoints,
            GeoPointStamped, GeoPointStamped, GeoPointStamped, GeoPointStamped)
            corresponding to the flyzones, search grid, waypoints, air drop
            position, off axis object location, the emergent object location,
            and the home position.

        Raises:
            Timeout: On Timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def get_all_objects(self):
        """Returns first 100 submitted objects.

        Returns:
            dict: Object IDs to object data (dict).

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def get_object(self, id):
        """Returns object with matching ID.

        Args:
            id: Object ID.

        Returns:
            dict: The object data.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def post_object(self, json_object):
        """Uploads new object for submission.

        Args:
            json_object: Object as a JSON string.

        Returns:
            Object ID.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def put_object(self, id, json_object):
        """Updates object information.

        Args:
            id: Object ID.
            json_object: Object as a JSON string.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def delete_object(self, id):
        """Deletes object with matching ID.

        Args:
            id: Object ID.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def post_object_image(self, id, png):
        """Adds or updates object image thumbnail as a compressed PNG.

        Args:
            id: Object ID.
            png: Object PNG image from a file.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            CvBridgeError: On image conversion failure.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def get_object_image(self, id):
        """Retrieves object image thumbnail.

        Args:
            id: Object ID.

        Returns:
            A ROS Image message.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            CvBridgeError: On image conversion failure.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def delete_object_image(self, id):
        """Deletes object image thumbnail.

        Args:
            id: Object ID.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        raise NotImplementedError


class InteroperabilityClient(BaseClient):

    """InteroperabilityClient.

    Attributes:
        url: Base URL of the Interoperability server.
        session: Requests session.
        timeout: Timeout in seconds for individual requests.
    """

    # Endpoint paths.
    LOGIN_PATH = "/api/login"
    MISSIONS_PATH = "/api/missions"
    MISSIONS_FORMAT_PATH = "/api/missions/{:d}"
    OBSTACLES_PATH = "/api/obstacles"
    TELEMETRY_PATH = "/api/telemetry"
    OBJECTS_PATH = "/api/odlcs"
    OBJECTS_FORMAT_PATH = "/api/odlcs/{:d}"
    OBJECTS_IMAGE_FORMAT_PATH = "/api/odlcs/{:d}/image"

    def __init__(self, url, username, password, timeout=1.0, verify=True):
        """Initializes an InteroperabilityClient.

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
        rate = rospy.Rate(1)
        while not reachable and not rospy.is_shutdown():
            try:
                response = requests.get(
                    self.url, timeout=self.timeout, verify=self.verify)
                response.raise_for_status()
                reachable = response.ok
            except requests.ConnectionError:
                rospy.logwarn_throttle(5.0, "Waiting for server: {}".format(
                    self.url))
            except Exception as e:
                rospy.logerr_throttle(
                    5.0, "Unexpected error waiting for server: {}, {}".format(
                        self.url, e))

            if not reachable:
                rate.sleep()

    def login(self):
        """Authenticates with the server.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        method = "POST"
        uri = self.LOGIN_PATH
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
            GeoCylinderArrayStamped of stationary obstacles.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
        """
        response = self._get(self.OBSTACLES_PATH)
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
        self._post(self.TELEMETRY_PATH, data=dict_telem)

    def get_active_mission(self, frame):
        """Gets active mission.

        Args:
            frame: Frame ID.

        Returns
            A tuple of (FlyZoneArray, GeoPolygonStamped, GeoObjectArray,
            GeoPointStamped, GeoPointStamped, GeoPointStamped, GeoPointStamped)
            corresponding to the flyzones, search grid, waypoints,
            air drop position, off axis object location, the emergent object
            location, the home position

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
            LookupError: On no active missions found.
        """
        response = self._get(self.MISSIONS_PATH)
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
            air drop position, off axis object location, the emergent object
            location, the home position.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
        """
        response = self._get(self.MISSIONS_PATH)
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
            position, off axis object location, the emergent object location,
            and the home position.

        Raises:
            Timeout: On Timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
        """
        response = self._get(self.MISSIONS_FORMAT_PATH.format(id))
        mission = serializers.MissionDeserializer.from_dict(
            response.json(), frame)
        return mission

    def get_all_objects(self):
        """Returns first 100 submitted objects.

        Returns:
            dict: Object IDs to object data (dict).

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
        """
        response = self._get(self.OBJECTS_PATH)
        objects = {t["id"]: t for t in response.json()}
        return objects

    def get_object(self, id):
        """Returns object with matching ID.

        Args:
            id: Object ID.

        Returns:
            dict: The object data.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            ValueError: On JSON decoding failure.
        """
        response = self._get(self.OBJECTS_FORMAT_PATH.format(id))
        return response.json()

    def post_object(self, json_object):
        """Uploads new object for submission.

        Args:
            json_object: Object as a JSON string.

        Returns:
            Object ID.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        response = self._post(self.OBJECTS_PATH, data=json_object)
        return response.json()["id"]

    def put_object(self, id, json_object):
        """Updates object information.

        Args:
            id: Object ID.
            json_object: Object as a JSON string.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        self._put(self.OBJECTS_FORMAT_PATH.format(id), data=json_object)

    def delete_object(self, id):
        """Deletes object with matching ID.

        Args:
            id: Object ID.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        self._delete(self.OBJECTS_FORMAT_PATH.format(id))

    def post_object_image(self, id, png):
        """Adds or updates object image thumbnail as a compressed PNG.

        Args:
            id: Object ID.
            png: Object PNG image from a file.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            CvBridgeError: On image conversion failure.
        """
        self._post(self.OBJECTS_IMAGE_FORMAT_PATH.format(id), data=png)

    def get_object_image(self, id):
        """Retrieves object image thumbnail.

        Args:
            id: Object ID.

        Returns:
            A ROS Image message.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
            CvBridgeError: On image conversion failure.
        """
        response = self._get(self.OBJECTS_IMAGE_FORMAT_PATH.format(id))
        img = serializers.ObjectImageSerializer.from_raw(response.content)
        return img

    def delete_object_image(self, id):
        """Deletes object image thumbnail.

        Args:
            id: Object ID.

        Raises:
            Timeout: On timeout.
            HTTPError: On request failure.
            ConnectionError: On connection failure.
        """
        self._delete(self.OBJECTS_IMAGE_FORMAT_PATH.format(id))

    def download_mission_info(self, path):
        """Downloads all mission information in a format readable by the
        OfflineInteroperabilityClient.

        Args:
            path: Target download path.
        """
        missions = self._get(self.MISSIONS_PATH)
        mission_path = os.path.join(path,
                                    OfflineInteroperabilityClient.MISSIONS_PATH)
        with open(mission_path, "wb") as f:
            f.write(json.dumps(missions.json(), indent=4, sort_keys=True))

        obstacles = self._get(self.OBSTACLES_PATH)
        obstacles_path = os.path.join(
            path, OfflineInteroperabilityClient.OBSTACLES_PATH)
        with open(obstacles_path, "wb") as f:
            f.write(json.dumps(obstacles.json(), indent=4, sort_keys=True))


class OfflineInteroperabilityClient(BaseClient):

    """OfflineInteroperabilityClient.

    Attributes:
        path: Path to root directory that stores all mission information.
    """

    # File paths.
    MISSIONS_PATH = "missions.json"
    OBSTACLES_PATH = "obstacles.json"

    def __init__(self, path, *args, **kwargs):
        """Initializes an OfflineInteroperabilityClient.

        Args:
            path: Path to root directory that stores all mission information.
            *args: Additional positional arguments, ignored.
            **kwargs: Additional key-word arguments, ignored.
        
        Raises:
            IOError: On missions or obstacles files not found.
            JSONDecodeError: On JSON deserialization error.
        """
        # Verify paths.
        self.path = path
        missions_path = os.path.join(path, self.MISSIONS_PATH)
        obstacles_path = os.path.join(path, self.OBSTACLES_PATH)
        if not os.path.isdir(path):
            raise IOError("No such directory: {}".format(path))
        if not os.path.exists(missions_path):
            raise IOError("No such file: {}".format(missions_path))
        if not os.path.exists(obstacles_path):
            raise IOError("No such file: {}".format(obstacles_path))

        # Load mission information.
        with open(missions_path, "rb") as f:
            self._missions = json.loads(f.read())
        with open(obstacles_path, "rb") as f:
            self._obstacles = json.loads(f.read())

    def wait_for_server(self):
        """Waits until interoperability server is reachable.
        
        Note: Doesn't do anything since the server is offline.
        """
        pass

    def login(self):
        """Authenticates with the server.
        
        Note: Always succeeds since there is no server.
        """
        pass

    def get_obstacles(self, frame, lifetime):
        """Returns obstacles as Markers.

        Args:
            frame: Frame ID of every Marker.
            lifetime: Lifetime of every Marker in seconds.

        Returns:
            GeoCylinderArrayStamped of stationary obstacles.
        """
        return serializers.ObstaclesDeserializer.from_dict(
            self._obstacles, frame, lifetime)

    def post_telemetry(self, navsat_msg, compass_msg):
        """Uploads telemetry information to Interoperability server.

        Note: This does nothing as the server is not connected.

        Args:
            navsat_msg: sensor_msgs/NavSatFix message.
            compass_msg: std_msgs/Float64 message in degrees.
        """
        pass

    def get_active_mission(self, frame):
        """Gets active mission.

        Args:
            frame: Frame ID.

        Returns
            A tuple of (FlyZoneArray, GeoPolygonStamped, GeoObjectArray,
            GeoPointStamped, GeoPointStamped, GeoPointStamped, GeoPointStamped)
            corresponding to the flyzones, search grid, waypoints,
            air drop position, off axis object location, the emergent object
            location, the home position

        Raises:
            LookupError: On no active missions found.
        """
        for m in self._missions:
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
            air drop position, off axis object location, the emergent object
            location, the home position.
        """
        return {
            m["id"]: serializers.MissionDeserializer.from_dict(m, frame)
            for m in self._missions
        }

    def get_mission(self, id, frame):
        """Returns mission with the matching ID.

        Args:
            id: Mission ID.
            frame: Frame ID.

        Returns:
            A tuple of (FlyZoneArray, GeoPolygonStamped, WayPoints,
            GeoPointStamped, GeoPointStamped, GeoPointStamped, GeoPointStamped)
            corresponding to the flyzones, search grid, waypoints, air drop
            position, off axis object location, the emergent object location,
            and the home position.

        Raises:
            LookupError: On mission ID not found.
        """
        for m in self._missions:
            if m["id"] == id:
                return serializers.MissionDeserializer.from_dict(m, frame)

        raise LookupError("Mission {:d} not found".format(id))

    def get_all_objects(self):
        """Returns first 100 submitted objects.

        Note: Does nothing as it relies on the local objects directory instead.

        Returns:
            dict: Object IDs to object data (dict).

        Raises:
            IOError: Always.
        """
        raise IOError("Cannot connect to remote server in offline mode")

    def get_object(self, id):
        """Returns object with matching ID.

        Note: Does nothing as it relies on the local objects directory instead.

        Args:
            id: Object ID.

        Returns:
            dict: The object data.

        Raises:
            IOError: Always.
        """
        raise IOError("Cannot connect to remote server in offline mode")

    def post_object(self, json_object):
        """Uploads new object for submission.

        Note: Does nothing as it relies on the local objects directory instead.

        Args:
            json_object: Object as a JSON string.

        Returns:
            Object ID.

        Raises:
            IOError: Always.
        """
        raise IOError("Cannot connect to remote server in offline mode")

    def put_object(self, id, json_object):
        """Updates object information.

        Note: Does nothing as it relies on the local objects directory instead.

        Args:
            id: Object ID.
            json_object: Object as a JSON string.

        Raises:
            IOError: Always.
        """
        raise IOError("Cannot connect to remote server in offline mode")

    def delete_object(self, id):
        """Deletes object with matching ID.

        Note: Does nothing as it relies on the local objects directory instead.

        Args:
            id: Object ID.

        Raises:
            IOError: Always.
        """
        raise IOError("Cannot connect to remote server in offline mode")

    def post_object_image(self, id, png):
        """Adds or updates object image thumbnail as a compressed PNG.

        Args:
            id: Object ID.
            png: Object PNG image from a file.

        Raises:
            IOError: Always.
        """
        raise IOError("Cannot connect to remote server in offline mode")

    def get_object_image(self, id):
        """Retrieves object image thumbnail.

        Args:
            id: Object ID.

        Returns:
            A ROS Image message.

        Raises:
            IOError: Always.
        """
        raise IOError("Cannot connect to remote server in offline mode")

    def delete_object_image(self, id):
        """Deletes object image thumbnail.

        Args:
            id: Object ID.

        Raises:
            IOError: Always.
        """
        raise IOError("Cannot connect to remote server in offline mode")
