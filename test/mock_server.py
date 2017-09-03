# -*- coding: utf-8 -*-
"""Interoperability mock server for testing."""

import json
import responses
from interop import serializers


class InteroperabilityMockServer(object):

    """Interoperability mock server.

    This should be used as a context manager using the `with` operator as
    follows:

        with InteroperabilityMockServer(url) as server:
            # Set up server responses.
            server.set_root_response()
            ...

            # Use InteroperabilityClient as usual.
            client = InteroperabilityClient(*args)
            client.wait_for_server()
            ...

    The server must be setup with the available `set_*_response()` methods for
    each request you're planning to make. If a request whose response has been
    set up is not made, the InteroperabilityMockServer will raise an
    AssertionError.
    """

    def __init__(self, url):
        """Constructs InteroperabilityMockServer.

        Args:
            url (str): Interoperability server base URL
                (e.g. http://127.0.0.1:8000)

        Returns:
            InteroperabilityServer.
        """
        self.url = url[:-1] if url.endswith('/') else url
        self.rsps = responses.RequestsMock()

    def __enter__(self):
        """Enters server context manager."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Exists server context manager.

        Raises:
            AssertionError: If a set up response was not requested.
        """
        # Avoid overwriting exception if there already is one.
        if not exc_val:
            self.stop()

    def start(self):
        """Starts mock server."""
        self.rsps.__enter__()

    def stop(self):
        """Stops mock server."""
        self.rsps.__exit__()

    def set_root_response(self, code=200):
        """Sets mock GET / response.

        This is needed for the `InteroperabilityClient.wait_for_server()` to
        work.

        Args:
            code (int): Status code to respond with.
        """
        self.rsps.add(responses.GET, self.url, status=code)

    def set_login_response(self, success=True):
        """Sets mock POST /api/login response.

        Args:
            success (bool): Whether the authentication attempt should be
                successful.
        """
        code = 200 if success else 400
        self.rsps.add(
            responses.POST,
            self.url + "/api/login",
            status=code,
            body="Login Successful." if success else "")

    def set_get_server_info_response(self,
                                     message,
                                     message_timestamp,
                                     server_time,
                                     code=200):
        """Sets mock GET /api/server_info response.

        Args:
            message (str): Message string in response.
            message_timestamp (str): Message timestamp in ISO8601.
            server_time (str): Server time in ISO8601.
            code (int): Status code to respond with.
        """
        content = json.dumps({
            "message": message,
            "message_timestamp": message_timestamp,
            "server_time": server_time
        })

        self.rsps.add(
            responses.GET,
            self.url + "/api/server_info",
            status=code,
            body=content if code == 200 else "",
            content_type="application/json")

    def set_get_obstacles_response(self, obstacles, code=200):
        """Sets mock GET /api/obstacles response.

        Args:
            obstacles (dict): Obstacles to respond with.
            code (int): Status code to respond with.
        """
        content = json.dumps(obstacles)
        self.rsps.add(
            responses.GET,
            self.url + "/api/obstacles",
            status=code,
            body=content if code == 200 else "",
            content_type="application/json")

    def set_telemetry_response(self, code=200):
        """Sets mock POST /api/telemetry response.

        Args:
            code (int): Status code to respond with.
        """
        self.rsps.add(
            responses.POST,
            self.url + "/api/telemetry",
            status=code,
            body="UAS Telemetry Successfully Posted." if code == 200 else "")

    def set_post_object_response(self, object_, id, user=1, code=200):
        """Sets mock POST /api/odlcs response.

        Args:
            object_ (dict): Target to post.
            id (int): Target ID to return.
            user (int): User number to respond with.
            code (int): Status code to respond with.
        """
        object_.update({"id": id, "user": user})
        content = json.dumps(object_)

        self.rsps.add(
            responses.POST,
            self.url + "/api/odlcs",
            status=code,
            body=content if code == 200 else "",
            content_type="application/json")

    def set_get_objects_response(self, objects, code=200):
        """Sets mock GET /api/odlcs and GET /api/odlcs/<id> responses.

        Args:
            objects (list): List of objects.
            code (int): Status code to respond with.
        """
        content = json.dumps(objects)

        # Add all objects.
        self.rsps.add(
            responses.GET,
            self.url + "/api/odlcs",
            status=code,
            body=content if code == 200 else "",
            content_type="application/json")

        # Add individual objects.
        for t in objects:
            self.rsps.add(
                responses.GET,
                "{}/api/odlcs/{:d}".format(self.url, t["id"]),
                body=json.dumps(t) if code == 200 else "",
                content_type="application/json")

    def set_put_object_response(self, id, object_, user=1, code=200):
        """Sets mock PUT /api/odlcs/<id> response.

        Args:
            id (int): Target ID.
            object_ (dict): Target data to update with.
            user (int): User number to respond with.
            code (int): Status code to respond with.
        """
        object_.update({"id": id, "user": user})
        content = json.dumps(object_)

        self.rsps.add(
            responses.PUT,
            "{}/api/odlcs/{:d}".format(self.url, id),
            status=code,
            body=content if code == 200 else "",
            content_type="application/json")

    def set_delete_object_response(self, id, code=200):
        """Sets mock DELETE /api/odlcs/<id> response.

        Args:
            id (int): Target ID.
            code (int): Status code to respond with.
        """
        self.rsps.add(
            responses.DELETE,
            "{}/api/odlcs/{:d}".format(self.url, id),
            status=code,
            body="Target deleted." if code == 200 else "")

    def set_post_object_image_response(self, id, code=200):
        """Sets mock POST /api/odlcs/<id>/image response.

        Args:
            id (int): Target ID.
            code (int): Status code to respond with.
        """
        self.rsps.add(
            responses.POST,
            "{}/api/odlcs/{:d}/image".format(self.url, id),
            body="Image uploaded." if code == 200 else "",
            status=code)

    def set_get_object_image_response(self, id, image, content_type, code=200):
        """Sets mock GET /api/odlcs/<id>/image response.

        Args:
            id (int): Target ID.
            image (str): Binary image string.
            content_type (str): Image type, either "image/jpeg" or "image/png".
            code (int): Status code to respond with.
        """
        self.rsps.add(
            responses.GET,
            "{}/api/odlcs/{:d}/image".format(self.url, id),
            body=image if code == 200 else "",
            status=code,
            content_type=content_type)

    def set_delete_object_image_response(self, id, code=200):
        """Sets mock DELETE /api/odlcs/<id>/image response.

        Args:
            id (int): Target ID.
            code (str): Status code to respond with.
        """
        self.rsps.add(
            responses.DELETE,
            "{}/api/odlcs/{:d}/image".format(self.url, id),
            body="Image deleted." if code == 200 else "",
            status=code)
