# -*- coding: utf-8 -*-

"""Asynchronous Interoperability HTTP Client."""

import urllib
import serializers
from tornado.escape import json_decode
from httpclient_session import Session
from tornado.gen import coroutine, Return
from tornado.httpclient import AsyncHTTPClient, HTTPRequest


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
        self.session = Session(AsyncHTTPClient)

    @coroutine
    def authenticate(self, username, password):
        """Authenticates with the Interoperability server with given
        credentials.

        Args:
            username: Interoperability server username.
            password: Interoperability server password.

        Raises:
            HTTPError: On failure or timeout.
        """
        yield self._post("/api/login",
                         body={"username": username, "password": password})

    @coroutine
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
        response = yield self.session.fetch(request)
        raise Return(response)

    @coroutine
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
        response = yield self.session.fetch(request)
        raise Return(response)

    @coroutine
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
        response = yield self.session.fetch(request)
        raise Return(response)

    @coroutine
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
        response = yield self.session.fetch(request)
        raise Return(response)

    @coroutine
    def get_server_info(self):
        """Returns server information.

        Returns:
            (std_msgs/String, std_msgs/Time, std_msgs/Time) tuple.
            The first is the server message, the second is the message
            timestamp and the last is the server time.
        """
        response = yield self._get("/api/server_info")
        json = json_decode(response.body)
        raise Return(serializers.ServerInfoDeserializer.from_json(json))


if __name__ == "__main__":
    from tornado.ioloop import IOLoop

    # Initialize client.
    client = InteroperabilityClient("http://localhost:80")

    @coroutine
    def authenticate():
        """Authenticate with server."""
        yield client.authenticate("testadmin", "testpass")

    @coroutine
    def print_server_info():
        """Print out server information."""
        r = yield client.get_server_info()
        print(r)

    # Test out authentication and basic request synchronously.
    IOLoop.current().run_sync(authenticate)
    IOLoop.current().run_sync(print_server_info)
