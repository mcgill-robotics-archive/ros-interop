#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Interoperability Objects ROS Server."""

import os
import sys
import json
import rospy
import errno
import datetime
import interop.srv
from cv_bridge import CvBridgeError
from interop import serializers, local_objects
from std_srvs.srv import Trigger, TriggerResponse
from interop import InteroperabilityClient, OfflineInteroperabilityClient


def trigger_exception_handler(*expected_exception_types):
    """Helper decorator for std_srvs/Trigger ROS services.

    Args:
        *expected_exception_types: All exception types that are expected to
            possibly occur. Any exception of one of these types will have
            response.success = False and will be logged with an ERROR log level.
            Other exceptions will not be caught, but will be automatically
            logged and show a stacktrace. They will not crash the service.

    Returns:
        Decorator that returns a TriggerResponse.
    """

    def decorator(f):

        def wrapper(*args, **kwargs):
            response = TriggerResponse()

            try:
                f(*args, **kwargs)
                response.success = True
            except expected_exception_types as e:
                rospy.logerr(e.message)
                response.success = False
                response.message = e.message

            return response

        return wrapper

    return decorator


class ObjectsServer(object):

    """Represents the ROS node for adding, updating, and deleting objects
    and images.
    Objects and images are stored locally and periodically synced
    to the interop server.
    """

    def __init__(self, objects_dir):
        """Initialize the objects server.

        Args:
            objects_dir (interop.local_objects.ObjectsDirectory):
                The directory used to store the object files and images.
        """
        self.objects_dir = objects_dir

    def add_object(self, req):
        """Handles AddObject service requests.

        Args:
            req: AddObjectRequest message.

        Returns:
            AddObjectResponse.
        """
        response = interop.srv.AddObjectResponse()

        dict_object = serializers.ObjectSerializer.from_msg(req.object)
        json_object = json.dumps(dict_object)

        try:
            file_id = self.objects_dir.add_object(json_object)
        except IOError as e:
            rospy.logerr(e)
            response.success = False
        except Exception as e:
            rospy.logfatal(e)
            response.success = False
        else:
            response.id = file_id
            response.success = True

        return response

    def get_object(self, req):
        """Handles GetObject service requests.

        Args:
            req: GetObjectRequest message.

        Returns:
            GetObjectResponse.
        """
        response = interop.srv.GetObjectResponse()

        try:
            json_object = self.objects_dir.get_object(req.id)
        except (KeyError, IOError) as e:
            rospy.logerr("Could not get object: {}".format(e))
            response.success = False
        except Exception as e:
            rospy.logfatal(e)
            response.success = False
        else:
            dict_object = json.loads(json_object)
            response.object = serializers.ObjectSerializer.from_dict(
                dict_object)
            response.success = True

        return response

    def update_object(self, req):
        """Handles UpdateObject service requests.

        Args:
            req: UpdateObjectRequest message.

        Returns:
            UpdateObjectResponse.
        """
        response = interop.srv.UpdateObjectResponse()

        dict_object = serializers.ObjectSerializer.from_msg(req.object)
        json_object = json.dumps(dict_object)

        try:
            self.objects_dir.update_object(req.id, json_object)
        except (KeyError, IOError) as e:
            rospy.logerr("Could not update object: {}".format(e))
            response.success = False
        except Exception as e:
            rospy.logfatal(e)
            response.success = False
        else:
            response.success = True

        return response

    def delete_object(self, req):
        """Handles DeleteObject service requests.

        Args:
            req: DeleteObjectRequest message.

        Returns:
            DeleteObjectResponse.
        """
        response = interop.srv.DeleteObjectResponse()

        try:
            self.objects_dir.delete_object(req.id)
        except (KeyError, OSError) as e:
            rospy.logerr("Could not delete object: {}".format(e))
            response.success = False
        except Exception as e:
            rospy.logfatal(e)
            response.success = False
        else:
            response.success = True

        return response

    def get_all_objects(self, req):
        """Handles GetAllObjects service requests.

        Args:
            req: GetAllObjectsRequest message.

        Returns:
            GetAllObjectsResponse.
        """
        response = interop.srv.GetAllObjectsResponse()

        try:
            json_objects = self.objects_dir.get_all_objects()
        except IOError as e:
            rospy.logerr("Could not get all objects: {}".format(e))
            response.success = False
        except Exception as e:
            rospy.logfatal(e)
            response.success = False
        else:
            for str_file_id, json_object in json_objects.iteritems():
                file_id = int(str_file_id)
                dict_object = json.loads(json_object)
                ros_object = serializers.ObjectSerializer.from_dict(dict_object)

                response.ids.append(file_id)
                response.objects.append(ros_object)

            response.success = True

        return response

    def set_object_image(self, req, compress=False):
        """Handles SetObjectImage service requests.

        Args:
            req: SetObjectImageRequest/SetObjectCompressedImageRequest message.
            compress: Whether to return a compressed image or not.

        Returns:
            SetObjectImageResponse or SetObjectCompressedImageResponse.
        """
        if compress:
            response = interop.srv.SetObjectCompressedImageResponse()
        else:
            response = interop.srv.SetObjectImageResponse()

        try:
            png_image = serializers.ObjectImageSerializer.from_msg(req.image)
        except CvBridgeError as e:
            rospy.logerr(e)
            response.success = False
        except Exception as e:
            rospy.logfatal(e)
            response.success = False
        else:
            try:
                self.objects_dir.set_object_image(req.id, png_image)
            except (KeyError, IOError) as e:
                rospy.logerr("Could not set object image: {}".format(e))
                response.success = False
            else:
                response.success = True

        return response

    def get_object_image(self, req, compress=False):
        """Handles GetObjectImage service requests.

        Args:
            req: GetObjectImageRequest/GetObjectCompressedImageRequest message.
            compress: Whether to return a compressed image or not.

        Returns:
            GetObjectImageResponse or GetObjectCompressedImageResponse.
        """
        if compress:
            response = interop.srv.GetObjectCompressedImageResponse()
        else:
            response = interop.srv.GetObjectImageResponse()

        try:
            png = self.objects_dir.get_object_image(req.id)
        except (KeyError, IOError) as e:
            rospy.logerr("Could not get object image: {}".format(e))
            response.success = False
        except Exception as e:
            rospy.logfatal(e)
            response.success = False
        else:
            try:
                response.image = serializers.ObjectImageSerializer.from_raw(
                    png, compress)
            except CvBridgeError as e:
                rospy.logerr(e)
                response.success = False
            else:
                response.success = True

        return response

    def delete_object_image(self, req):
        """Handles DeleteObjectImage service requests.

        Args:
            req: DeleteObjectImageRequest message.

        Returns:
            DeleteObjectImageResponse.
        """
        response = interop.srv.DeleteObjectImageResponse()

        try:
            self.objects_dir.delete_object_image(req.id)
        except (KeyError, IOError) as e:
            rospy.logerr("Could not delete object image: {}".format(e))
            response.success = False
        except Exception as e:
            rospy.logfatal(e)
            response.success = False
        else:
            response.success = True

        return response

    @trigger_exception_handler()
    def reload_all_objects(self, req):
        """Reloads all remote objects.

        Args:
            req: TriggerRequest message.

        Returns:
            TriggerResponse.
        """
        rospy.logwarn("Reloading all remote objects...")
        self.objects_dir.load_all_remote_objects()

    @trigger_exception_handler()
    def clear_all_objects(self, req):
        """Reloads all remote objects.

        Args:
            req: TriggerRequest message.

        Returns:
            TriggerResponse.
        """
        rospy.logwarn("Clearing all objects...")
        self.objects_dir.clear_all_objects()
        rospy.loginfo("Cleared all objects successfully")

    def sync(self, rospy_timer_event):
        """Handles calls from rospy.Timer to sync the local objects and images
        to the interop server.

        Args:
            rospy_timer_event (rospy.TimerEvent): Unused formal parameter
                necessary for making this function work as a callback for
                rospy.Timer.
        """
        self.objects_dir.sync()


def get_objects_path(objects_root):
    """"Gets a new objects diretory.

    Args:
        objects_root: Parent directory of objects directory.

    Returns:
        Path to objects directory.
    """
    # Set folder name to timestamp (YYYY-mm-DD-hh-MM-ss)
    dirname = "{:%Y-%m-%d-%H-%M-%S}".format(datetime.datetime.now())

    # Set directory for objects: /<objects_root>/<dirname>/.
    objects_root = os.path.expanduser(objects_root)
    objects_path = os.path.join(objects_root, dirname)

    return objects_path


def create_objects_path(objects_path):
    """Creates the objects directory.

    Args:
        objects_path: Path to objects directory.

    Raises:
        OSError: If the directory could not be created.
    """
    try:
        os.makedirs(objects_path)
    except OSError as e:
        # Allow reusing the same directory.
        if e.errno != errno.EEXIST:
            raise


def symlink_objects_path_to_latest(objects_path):
    """Symlinks 'latest' to the given directory.

    Args:
        objects_path: Path to objects directory.
    """
    objects_root = os.path.dirname(objects_path)
    path_to_symlink = os.path.join(objects_root, "latest")

    try:
        os.symlink(objects_path, path_to_symlink)
    except OSError as e:
        # Replace the old symlink if an old symlink with the same
        # name exists.
        if e.errno == errno.EEXIST:
            os.remove(path_to_symlink)
            os.symlink(objects_path, path_to_symlink)
        else:
            rospy.logerr(
                "Could not create symlink to the latest objects directory")


if __name__ == "__main__":
    # Initialize node.
    rospy.init_node("objects")

    # Get server connection information.
    offline = rospy.get_param("~offline")
    if offline:
        base_path = rospy.get_param("~base_path")
        client = OfflineInteroperabilityClient(base_path)
        rospy.logwarn("Running in OFFLINE mode")
    else:
        base_url = rospy.get_param("~base_url")
        timeout = rospy.get_param("~timeout")
        verify = rospy.get_param("~verify")
        client = InteroperabilityClient.from_env(base_url, timeout, verify)

    # Get other ROS parameters.
    objects_root = rospy.get_param("~objects_root")
    update_period = rospy.get_param("~interop_update_period")

    # Wait for server to be reachable, then login.
    client.wait_for_server()
    try:
        client.login()
    except Exception as e:
        rospy.logfatal(e)
        sys.exit(1)

    # Initialize a directory for storing the objects.
    try:
        # Set up directory.
        objects_path = get_objects_path(objects_root)
        rospy.loginfo("Storing object files in {}".format(objects_path))
        create_objects_path(objects_path)
        symlink_objects_path_to_latest(objects_path)
        objects_dir = local_objects.ObjectsDirectory(objects_path, client,
                                                     offline)
    except OSError as e:
        rospy.logfatal(e)
        raise

    if not offline:
        try:
            # Sync up the objects directory.
            rospy.loginfo("Loading all remote objects...")
            objects_dir.load_all_remote_objects()
        except Exception as e:
            rospy.logfatal(e)
            raise

    # Set up the objects server.
    objects_server = ObjectsServer(objects_dir)

    # Set up a timer to periodically update the objects and images
    # on the interop server.
    rospy.Timer(rospy.Duration(update_period), objects_server.sync)

    # Initialize object ROS services.
    rospy.Service("~add", interop.srv.AddObject, objects_server.add_object)
    rospy.Service("~get", interop.srv.GetObject, objects_server.get_object)
    rospy.Service("~update", interop.srv.UpdateObject,
                  objects_server.update_object)
    rospy.Service("~delete", interop.srv.DeleteObject,
                  objects_server.delete_object)
    rospy.Service("~all", interop.srv.GetAllObjects,
                  objects_server.get_all_objects)

    # Initialize object image ROS services.
    rospy.Service("~image/set", interop.srv.SetObjectImage,
                  objects_server.set_object_image)
    rospy.Service("~image/get", interop.srv.GetObjectImage,
                  objects_server.get_object_image)
    rospy.Service("~image/delete", interop.srv.DeleteObjectImage,
                  objects_server.delete_object_image)

    # Initialize object compressed image ROS services.
    rospy.Service("~image/compressed/set", interop.srv.SetObjectCompressedImage,
                  lambda r: objects_server.set_object_image(r, True))
    rospy.Service("~image/compressed/get", interop.srv.GetObjectCompressedImage,
                  lambda r: objects_server.get_object_image(r, True))

    # Initialize objects syncing ROS services.
    rospy.Service("~clear", Trigger, objects_server.clear_all_objects)
    rospy.Service("~reload", Trigger, objects_server.reload_all_objects)

    rospy.spin()
