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
            url: Interoperability server base URL (e.g. http://127.0.0.1:8000)

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
            code: Status code to respond with.
        """
        self.rsps.add(responses.GET, self.url, status=code)

    def set_login_response(self, success=True):
        """Sets mock POST /api/login response.

        Args:
            success: Whether the authentication attempt should be successful.
        """
        code = 200 if success else 400
        self.rsps.add(responses.POST, self.url + "/api/login", status=code,
                      body="Login Successful." if success else "")

    def set_get_server_info_response(self, message, message_timestamp,
                                     server_time, code=200):
        """Sets mock GET /api/server_info response.

        Args:
            message: Message string in response.
            message_timestamp: Message timestamp in ISO8601.
            server_time: Server time in ISO8601.
            code: Status code to respond with.
        """
        content = json.dumps({
            "message": message,
            "message_timestamp": message_timestamp,
            "server_time": server_time
        })

        self.rsps.add(responses.GET, self.url + "/api/server_info",
                      status=code, body=content,
                      content_type="application/json")

    def set_get_obstacles_response(self, obstacles, code=200):
        """Sets mock GET /api/obstacles response.

        Args:
            obstacles: JSON serialized obstacles to respond with.
            code: Status code to respond with.
        """
        content = json.dumps(obstacles)
        self.rsps.add(responses.GET, self.url + "/api/obstacles", status=code,
                      body=content, content_type="application/json")

    def set_telemetry_response(self, code=200):
        """Sets mock POST /api/telemetry response.

        Args:
            code: Status code to respond with.
        """
        self.rsps.add(responses.POST, self.url + "/api/telemetry", status=code,
                      body="UAS Telemetry Successfully Posted."
                           if code == 200 else "")

    def set_post_target_response(self, target, id, user=1, code=200):
        """Sets mock POST /api/targets response.

        Args:
            target: Target message to post.
            id: Target ID to return.
            user: User number to respond with.
            code: Status code to respond with.
        """
        target_json = serializers.TargetSerializer.from_msg(target)
        target_json.update({"id": id, "user": user})
        content = json.dumps(target_json)

        self.rsps.add(responses.POST, self.url + "/api/targets", status=code,
                      body=content, content_type="application/json")

    def set_get_targets_response(self, targets, code=200):
        """Sets mock GET /api/targets and GET /api/targets/<id> responses.

        Args:
            targets: JSON serialized target list.
            code: Status code to respond with.
        """
        content = json.dumps(targets)

        # Add all targets.
        self.rsps.add(responses.GET, self.url + "/api/targets", status=code,
                      body=content, content_type="application/json")

        # Add individual targets.
        for t in targets:
            self.rsps.add(responses.GET,
                          "{}/api/targets/{:d}".format(self.url, t["id"]),
                          body=json.dumps(t), content_type="application/json")

    def set_put_target_response(self, id, target, user=1, code=200):
        """Sets mock PUT /api/targets/<id> response.

        Args:
            id: Target ID.
            target: Target message to update with.
            user: User number to respond with.
            code: Status code to respond with.
        """
        target_json = serializers.TargetSerializer.from_msg(target)
        target_json.update({"id": id, "user": user})
        content = json.dumps(target_json)

        self.rsps.add(responses.PUT,
                      "{}/api/targets/{:d}".format(self.url, id), status=code,
                      body=content, content_type="application/json")

    def set_delete_target_response(self, id, code=200):
        """Sets mock DELETE /api/targets/<id> response.

        Args:
            id: Target ID.
            code: Status code to respond with.
        """
        self.rsps.add(responses.DELETE,
                      "{}/api/targets/{:d}".format(self.url, id), status=code,
                      body="Target deleted." if code == 200 else "")

    def set_post_target_image_response(self, id, code=200):
        """Sets mock POST /api/targets/<id>/image response.

        Args:
            id: Target ID.
            code: Status code to respond with.
        """
        self.rsps.add(responses.POST,
                      "{}/api/targets/{:d}/image".format(self.url, id),
                      body="Image uploaded." if code == 200 else "",
                      status=code)

    def set_get_target_image_response(self, id, image, content_type, code=200):
        """Sets mock GET /api/targets/<id>/image response.

        Args:
            id: Target ID.
            image: Binary image string.
            content_type: Image type, either "image/jpeg" or "image/png".
            code: Status code to respond with.
        """
        self.rsps.add(responses.GET,
                      "{}/api/targets/{:d}/image".format(self.url, id),
                      body=image if code == 200 else "",
                      status=code, content_type=content_type)

    def set_delete_target_image_response(self, id, code=200):
        """Sets mock DELETE /api/targets/<id>/image response.

        Args:
            id: Target ID.
            code: Status code to respond with.
        """
        self.rsps.add(responses.DELETE,
                      "{}/api/targets/{:d}/image".format(self.url, id),
                      body="Image deleted." if code == 200 else "",
                      status=code)
