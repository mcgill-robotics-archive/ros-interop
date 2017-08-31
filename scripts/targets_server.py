#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Interoperability Target ROS Server."""

import os
import sys
import json
import rospy
import errno
import datetime
import interop.srv
from cv_bridge import CvBridgeError
from interop import InteroperabilityClient
from interop import serializers, local_targets
from std_srvs.srv import Trigger, TriggerResponse


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


class TargetsServer(object):

    """Represents the ROS node for adding, updating, and deleting targets
    and images.
    Targets and images are stored locally and periodically synced
    to the interop server.
    """

    def __init__(self, targets_dir):
        """Initialize the targets server.

        Args:
            targets_dir (interop.local_targets.TargetsDirectory):
                The directory used to store the target files and images.
        """
        self.targets_dir = targets_dir

    def add_target(self, req):
        """Handles AddTarget service requests.

        Args:
            req: AddTargetRequest message.

        Returns:
            AddTargetResponse.
        """
        response = interop.srv.AddTargetResponse()

        dict_target = serializers.TargetSerializer.from_msg(req.target)
        json_target = json.dumps(dict_target)

        try:
            file_id = self.targets_dir.add_target(json_target)
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

    def get_target(self, req):
        """Handles GetTarget service requests.

        Args:
            req: GetTargetRequest message.

        Returns:
            GetTargetResponse.
        """
        response = interop.srv.GetTargetResponse()

        try:
            json_target = self.targets_dir.get_target(req.id)
        except (KeyError, IOError) as e:
            rospy.logerr("Could not get target: {}".format(e))
            response.success = False
        except Exception as e:
            rospy.logfatal(e)
            response.success = False
        else:
            dict_target = json.loads(json_target)
            response.target = serializers.TargetSerializer.from_dict(
                dict_target)
            response.success = True

        return response

    def update_target(self, req):
        """Handles UpdateTarget service requests.

        Args:
            req: UpdateTargetRequest message.

        Returns:
            UpdateTargetResponse.
        """
        response = interop.srv.UpdateTargetResponse()

        dict_target = serializers.TargetSerializer.from_msg(req.target)
        json_target = json.dumps(dict_target)

        try:
            self.targets_dir.update_target(req.id, json_target)
        except (KeyError, IOError) as e:
            rospy.logerr("Could not update target: {}".format(e))
            response.success = False
        except Exception as e:
            rospy.logfatal(e)
            response.success = False
        else:
            response.success = True

        return response

    def delete_target(self, req):
        """Handles DeleteTarget service requests.

        Args:
            req: DeleteTargetRequest message.

        Returns:
            DeleteTargetResponse.
        """
        response = interop.srv.DeleteTargetResponse()

        try:
            self.targets_dir.delete_target(req.id)
        except (KeyError, OSError) as e:
            rospy.logerr("Could not delete target: {}".format(e))
            response.success = False
        except Exception as e:
            rospy.logfatal(e)
            response.success = False
        else:
            response.success = True

        return response

    def get_all_targets(self, req):
        """Handles GetAllTargets service requests.

        Args:
            req: GetAllTargetsRequest message.

        Returns:
            GetAllTargetsResponse.
        """
        response = interop.srv.GetAllTargetsResponse()

        try:
            json_targets = self.targets_dir.get_all_targets()
        except IOError as e:
            rospy.logerr("Could not get all targets: {}".format(e))
            response.success = False
        except Exception as e:
            rospy.logfatal(e)
            response.success = False
        else:
            for str_file_id, json_target in json_targets.iteritems():
                file_id = int(str_file_id)
                dict_target = json.loads(json_target)
                ros_target = serializers.TargetSerializer.from_dict(dict_target)

                response.ids.append(file_id)
                response.targets.append(ros_target)

            response.success = True

        return response

    def set_target_image(self, req, compress=False):
        """Handles SetTargetImage service requests.

        Args:
            req: SetTargetImageRequest/SetTargetCompressedImageRequest message.
            compress: Whether to return a compressed image or not.

        Returns:
            SetTargetImageResponse or SetTargetCompressedImageResponse.
        """
        if compress:
            response = interop.srv.SetTargetCompressedImageResponse()
        else:
            response = interop.srv.SetTargetImageResponse()

        try:
            png_image = serializers.TargetImageSerializer.from_msg(req.image)
        except CvBridgeError as e:
            rospy.logerr(e)
            response.success = False
        except Exception as e:
            rospy.logfatal(e)
            response.success = False
        else:
            try:
                self.targets_dir.set_target_image(req.id, png_image)
            except (KeyError, IOError) as e:
                rospy.logerr("Could not set target image: {}".format(e))
                response.success = False
            else:
                response.success = True

        return response

    def get_target_image(self, req, compress=False):
        """Handles GetTargetImage service requests.

        Args:
            req: GetTargetImageRequest/GetTargetCompressedImageRequest message.
            compress: Whether to return a compressed image or not.

        Returns:
            GetTargetImageResponse or GetTargetCompressedImageResponse.
        """
        if compress:
            response = interop.srv.GetTargetCompressedImageResponse()
        else:
            response = interop.srv.GetTargetImageResponse()

        try:
            png = self.targets_dir.get_target_image(req.id)
        except (KeyError, IOError) as e:
            rospy.logerr("Could not get target image: {}".format(e))
            response.success = False
        except Exception as e:
            rospy.logfatal(e)
            response.success = False
        else:
            try:
                response.image = serializers.TargetImageSerializer.from_raw(
                    png, compress)
            except CvBridgeError as e:
                rospy.logerr(e)
                response.success = False
            else:
                response.success = True

        return response

    def delete_target_image(self, req):
        """Handles DeleteTargetImage service requests.

        Args:
            req: DeleteTargetImageRequest message.

        Returns:
            DeleteTargetImageResponse.
        """
        response = interop.srv.DeleteTargetImageResponse()

        try:
            self.targets_dir.delete_target_image(req.id)
        except (KeyError, IOError) as e:
            rospy.logerr("Could not delete target image: {}".format(e))
            response.success = False
        except Exception as e:
            rospy.logfatal(e)
            response.success = False
        else:
            response.success = True

        return response

    @trigger_exception_handler()
    def reload_all_targets(self, req):
        """Reloads all remote targets.

        Args:
            req: TriggerRequest message.

        Returns:
            TriggerResponse.
        """
        rospy.logwarn("Reloading all remote targets...")
        self.targets_dir.load_all_remote_targets()

    @trigger_exception_handler()
    def clear_all_targets(self, req):
        """Reloads all remote targets.

        Args:
            req: TriggerRequest message.

        Returns:
            TriggerResponse.
        """
        rospy.logwarn("Clearing all targets...")
        self.targets_dir.clear_all_targets()
        rospy.loginfo("Cleared all targets successfully")

    def sync(self, rospy_timer_event):
        """Handles calls from rospy.Timer to sync the local targets and images
        to the interop server.

        Args:
            rospy_timer_event (rospy.TimerEvent): Unused formal parameter
                necessary for making this function work as a callback for
                rospy.Timer.
        """
        self.targets_dir.sync()


def get_targets_path(targets_root):
    """"Gets a new targets diretory.

    Args:
        targets_root: Parent directory of targets directory.

    Returns:
        Path to targets directory.
    """
    # Set folder name to timestamp (YYYY-mm-DD-hh-MM-ss)
    dirname = "{:%Y-%m-%d-%H-%M-%S}".format(datetime.datetime.now())

    # Set directory for targets: /<targets_root>/<dirname>/.
    targets_root = os.path.expanduser(targets_root)
    targets_path = os.path.join(targets_root, dirname)

    return targets_path


def create_targets_path(targets_path):
    """Creates the targets directory.

    Args:
        targets_path: Path to targets directory.

    Raises:
        OSError: If the directory could not be created.
    """
    try:
        os.makedirs(targets_path)
    except OSError as e:
        # Allow reusing the same directory.
        if e.errno != errno.EEXIST:
            raise


def symlink_targets_path_to_latest(targets_path):
    """Symlinks 'latest' to the given directory.

    Args:
        targets_path: Path to targets directory.
    """
    targets_root = os.path.dirname(targets_path)
    path_to_symlink = os.path.join(targets_root, "latest")

    try:
        os.symlink(targets_path, path_to_symlink)
    except OSError as e:
        # Replace the old symlink if an old symlink with the same
        # name exists.
        if e.errno == errno.EEXIST:
            os.remove(path_to_symlink)
            os.symlink(targets_path, path_to_symlink)
        else:
            rospy.logerr(
                "Could not create symlink to the latest targets directory")


if __name__ == "__main__":
    # Initialize node.
    rospy.init_node("targets")

    # Get ROS parameters for client.
    base_url = rospy.get_param("~base_url")
    timeout = rospy.get_param("~timeout")
    verify = rospy.get_param("~verify")
    targets_root = rospy.get_param("~targets_root")
    update_period = rospy.get_param("~interop_update_period")

    # Initialize interoperability client.
    client = InteroperabilityClient.from_env(base_url, timeout, verify)

    # Wait for server to be reachable, then login.
    client.wait_for_server()
    try:
        client.login()
    except Exception as e:
        rospy.logfatal(e)
        sys.exit(1)

    # Initialize a directory for storing the targets.
    try:
        # Set up directory.
        targets_path = get_targets_path(targets_root)
        rospy.loginfo("Storing object files in {}".format(targets_path))
        create_targets_path(targets_path)
        symlink_targets_path_to_latest(targets_path)
        targets_dir = local_targets.TargetsDirectory(targets_path, client)
    except OSError as e:
        rospy.logfatal(e)
        raise

    try:
        # Sync up the targets directory.
        rospy.loginfo("Loading all remote targets...")
        targets_dir.load_all_remote_targets()
    except Exception as e:
        rospy.logfatal(e)
        raise

    # Set up the targets server.
    targets_server = TargetsServer(targets_dir)

    # Set up a timer to periodically update the targets and images
    # on the interop server.
    rospy.Timer(rospy.Duration(update_period), targets_server.sync)

    # Initialize target ROS services.
    rospy.Service("~add", interop.srv.AddTarget, targets_server.add_target)
    rospy.Service("~get", interop.srv.GetTarget, targets_server.get_target)
    rospy.Service("~update", interop.srv.UpdateTarget,
                  targets_server.update_target)
    rospy.Service("~delete", interop.srv.DeleteTarget,
                  targets_server.delete_target)
    rospy.Service("~all", interop.srv.GetAllTargets,
                  targets_server.get_all_targets)

    # Initialize target image ROS services.
    rospy.Service("~image/set", interop.srv.SetTargetImage,
                  targets_server.set_target_image)
    rospy.Service("~image/get", interop.srv.GetTargetImage,
                  targets_server.get_target_image)
    rospy.Service("~image/delete", interop.srv.DeleteTargetImage,
                  targets_server.delete_target_image)

    # Initialize target compressed image ROS services.
    rospy.Service("~image/compressed/set", interop.srv.SetTargetCompressedImage,
                  lambda r: targets_server.set_target_image(r, True))
    rospy.Service("~image/compressed/get", interop.srv.GetTargetCompressedImage,
                  lambda r: targets_server.get_target_image(r, True))

    # Initialize targets syncing ROS services.
    rospy.Service("~clear", Trigger, targets_server.clear_all_targets)
    rospy.Service("~reload", Trigger, targets_server.reload_all_targets)

    rospy.spin()
