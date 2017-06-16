# -*- coding: utf-8 -*-

import os
import json
import errno
import rospy
import os.path
import datetime
import threading
import serializers
from cv_bridge import CvBridgeError
from simplejson import JSONDecodeError
from requests.exceptions import ConnectionError, HTTPError, Timeout


class Target(object):

    """Represents a target and its image that is stored inside a local
    targets directory.
    Contains a sync function that syncs the target and its image to the
    interop server.
    """

    def __init__(self, targets_dir, file_id, data, client, interop_id=None):
        """Creates the target file with the specified data, inside the
        specified directory.

        Args:
            targets_dir (str): Absolute path to the parent directory
                of the target file.
            file_id (int): ID associated with this target.
            data (str): The target data that will be written into the target
                file.
            client (interop.InteroperabilityClient): Interoperability client
                that will be used to sync the target and its image to the
                server.
            interop_id (int): Remote ID associated with this target, optional.

        Raises:
            IOError: If the target file could not be written.
        """
        # Reentrant lock needed for sync where there are attempts to
        # acquire the lock twice.
        # Dual use: preventing simultaneous file IO, and preventing simultaneous
        # changes to the state variables (_needs_adding, _needs_updating, etc.).
        self.lock = threading.RLock()

        self.client = client
        self.targets_dir = targets_dir

        self.file_id = file_id
        # Also used to indicate presence on the server.
        self.interop_id = interop_id
        self.image_is_on_server = False

        # State variables used to decide how/what to sync to the interop server.
        # Intended to only to be accessed and modified using properties.
        self._needs_adding = False
        self._needs_updating = False
        self._needs_deleting = False
        self._image_needs_setting = False
        self._image_needs_deleting = False

        filename = str(self.file_id) + ".json"
        # self.target_path may become None when target is deleted locally.
        self.target_path = os.path.join(self.targets_dir, filename)
        self.image_path = None

        # Create the target file.
        with self.lock:
            try:
                with open(self.target_path, "w", 0) as f:
                    f.write(data)
            except IOError as e:
                raise

            self.needs_adding = interop_id is None

    @property
    def needs_adding(self):
        return self._needs_adding

    @needs_adding.setter
    def needs_adding(self, value):
        self._needs_adding = value

    @property
    def needs_updating(self):
        return self._needs_updating

    @needs_updating.setter
    def needs_updating(self, value):
        if value:
            # No need to update target if it has not been added.
            if self._needs_adding:
                self._needs_updating = False
            # No need to update target if it is to be deleted.
            elif self._needs_deleting:
                self._needs_updating = False
            else:
                self._needs_updating = True
        else:
            self._needs_updating = False

    @property
    def needs_deleting(self):
        return self._needs_deleting

    @needs_deleting.setter
    def needs_deleting(self, value):
        if value:
            # If the target is not on the the server,
            # there is no need to do anything.
            if self.interop_id is None:
                self._needs_adding = False
                self._needs_updating = False
                self._needs_deleting = False
            # If the target is on the server, there is no need to
            # do anything other than deleting from the server.
            else:
                self._needs_adding = False
                self._needs_updating = False
                self._needs_deleting = True

            # No need to do anything with the image as it will be
            # deleted together with the target anyway.
            self.image_needs_setting = False
            self.image_needs_deleting = False
        else:
            self._needs_deleting = False

    @property
    def image_needs_setting(self):
        return self._image_needs_setting

    @image_needs_setting.setter
    def image_needs_setting(self, value):
        # No need to set image if image will be deleted.
        if self._image_needs_deleting:
            self._image_needs_setting = False
        else:
            self._image_needs_setting = value

    @property
    def image_needs_deleting(self):
        return self._image_needs_deleting

    @image_needs_deleting.setter
    def image_needs_deleting(self, value):
        if value:
            # If the image is not on the server,
            # there is no need to do anything.
            if not self.image_is_on_server:
                self._image_needs_setting = False
                self._image_needs_deleting = False
            else:
                self._image_needs_setting = False
                self._image_needs_deleting = True
        else:
            self._image_needs_deleting = False

    def update(self, data):
        """Update this target.

        Args:
            data (str): The updated string to write into this target.

        Raises:
            IOError: If the path to the target file is not known,
                or if the target file could not be written.
        """
        with self.lock:
            if self.target_path is None:
                raise IOError("Could not update target for file_id {}. "
                    "Path to target file not known.".format(self.file_id))
            else:
                try:
                    with open(self.target_path, "w", 0) as f:
                        f.write(data)
                except IOError as e:
                    raise

                self.needs_updating = True

    def delete(self):
        """Delete this target and its associated image.
        Will silently fail if there is nothing to delete.

        Raises:
            OSError: If the path to the target file is not known,
                or if the target file or image could not be deleted.
        """
        with self.lock:
            # Delete target.
            if self.target_path is None:
                raise IOError("Could not delete target for file_id {}. "
                    "Path to target file not known.".format(self.file_id))
            else:
                try:
                    os.remove(self.target_path)
                except OSError as e:
                    raise

                self.target_path = None

            # Delete associated image.
            if self.image_path is not None:
                try:
                    os.remove(self.image_path)
                except OSError as e:
                    raise

                self.image_path = None

            self.needs_deleting = True

    def get(self):
        """Returns the content of the target file.

        Returns:
            str: The contents of the target file.

        Raises:
            IOError: If the path to the target file is not known, or if the
                file could not be read.
        """
        with self.lock:
            if self.target_path is None:
                raise IOError("Target file for file_id {} does not exist."
                    .format(self.file_id))
            else:
                try:
                    with open(self.target_path, "r") as f:
                        data = f.read()
                except IOError as e:
                    raise

                return data

    def set_image(self, png_image, needs_adding=True):
        """Associate an image with this target, or update the existing image.

        Args:
            png_image (str): The PNG image to be written.
            needs_adding (bool): Whether the image needs to be added to the
                server or not.

        Raises:
            IOError: If the image could not be written.
        """
        with self.lock:
            filename = str(self.file_id) + ".png"
            self.image_path = os.path.join(self.targets_dir, filename)

            try:
                with open(self.image_path, "wb", 0) as f:
                    f.write(png_image)
            except IOError as e:
                raise

            self.image_needs_setting = needs_adding

    def delete_image(self):
        """Delete the image associated with this target.
        Will silently fail if there is nothing to delete.

        Raises:
            OSError: If the path to the target image is not known,
                or if the target image could not be deleted.
        """
        with self.lock:
            if self.image_path is None:
                raise IOError("Could not delete image for file_id {}. "
                    "Path to image file not known.".format(self.file_id))
            else:
                try:
                    os.remove(self.image_path)
                except OSError as e:
                    raise

            self.image_needs_deleting = True

    def get_image(self):
        """Delete this target and its associated image.

        Raises:
            IOError: If the path to the image file is not known, or if the
                file could not be read.
        """
        with self.lock:
            if self.image_path is None:
                raise IOError("Could not get image file. "
                    "There is no image associated with file_id {}."
                    .format(self.file_id))
            else:
                try:
                    # png_image is expected to be of type str.
                    with open(self.image_path, "rb") as f:
                        png_image = f.read()
                except IOError as e:
                    raise

                return png_image

    def sync(self):
        """Syncs this target and its image to the interop server."""
        with self.lock:
            # TARGET FILE
            if self.needs_adding:
                try:
                    target = self.get()
                except IOError as e:
                    rospy.logerr(e)
                else:
                    try:
                        # Post target and record the interop_id.
                        self.interop_id = self.client.post_target(target)
                    except (ConnectionError, Timeout) as e:
                        rospy.logwarn(e)
                    except (JSONDecodeError, HTTPError) as e:
                        rospy.logerr(e)
                    else:
                        # No longer needs adding.
                        self.needs_adding = False

            # An interop id is needed to update.
            elif self.needs_updating and self.interop_id is not None:
                try:
                    target = self.get()
                except IOError as e:
                    rospy.logerr(e)
                else:
                    try:
                        self.client.put_target(self.interop_id, target)
                    except (ConnectionError, Timeout) as e:
                        rospy.logwarn(e)
                    except (JSONDecodeError, HTTPError) as e:
                        rospy.logerr(e)
                    else:
                        self.needs_updating = False

            elif self.needs_deleting and self.interop_id is not None:
                try:
                    self.client.delete_target(self.interop_id)
                except (ConnectionError, Timeout) as e:
                    rospy.logwarn(e)
                except (JSONDecodeError, HTTPError) as e:
                    rospy.logerr(e)
                else:
                    self.interop_id = None
                    self.image_is_on_server = False
                    self.needs_deleting = False

            # IMAGE FILE
            if self.image_needs_setting and self.interop_id is not None:
                try:
                    image = self.get_image()
                except IOError as e:
                    rospy.logerr(e)
                else:
                    try:
                        self.client.post_target_image(self.interop_id, image)
                    except (ConnectionError, Timeout) as e:
                        rospy.logwarn(e)
                    except (CvBridgeError, HTTPError) as e:
                        rospy.logerr(e)
                    else:
                        self.image_is_on_server = True
                        self.image_needs_setting = False

            elif (self.image_needs_deleting and self.image_is_on_server
                    and self.interop_id is not None):
                try:
                    self.client.delete_target_image(self.interop_id)
                except (ConnectionError, Timeout) as e:
                    rospy.logwarn(e)
                except (CvBridgeError, HTTPError) as e:
                    rospy.logerr(e)
                else:
                    self.image_is_on_server = False
                    self.image_needs_deleting = False

    def can_be_forgotten(self):
        """When a target is removed locally and on the interop server, it
        no longer has any use. References to a target may still exist elsewhere
        in program. This method helps to determine if those references can be
        deleted.

        Returns:
            True if the target object can be forgotten, False otherwise.
        """
        with self.lock:
            # If the target and/or its image is still stored locally.
            if self.target_path or self.image_path:
                return False

            # If there are still things to be done on the interop server.
            if (self.needs_adding or self.needs_updating or self.needs_deleting
                    or self.image_needs_setting or self.image_needs_deleting):
                return False

            # Otherwise, the target is useless and all references can be deleted.
            return True


class TargetsDirectory(object):

    """Represents the directory containing target files and images.
    Contains a sync function that syncs the contents of the directory to
    the interop server.
    """

    def __init__(self, targets_root, client):
        """Creates a directory for storing targets and images.

        Args:
            targets_root (str): Absolute path to the parent directory of
                the directory to be created.
            client (interop.InteroperabilityClient): Interoperability client that
                will be used to handle syncs to the interop server.

        Raises:
            OSError: If the directory could not be created.
        """
        self.lock = threading.Lock()

        # Create timestamp (YYYY-mm-DD-hh-MM-ss).
        timestamp = "{:%Y-%m-%d-%H-%M-%S}".format(datetime.datetime.now())

        # Perform shell expansion
        targets_root = os.path.expanduser(targets_root)

        # Create directory for targets (/<targets_root>/<timestamp>/).
        self.targets_dir = os.path.join(targets_root, timestamp)
        try:
            os.makedirs(self.targets_dir)
        except OSError as e:
            raise

        # Create symlink to directory.
        path_to_symlink = os.path.join(targets_root, 'latest')
        try:
            os.symlink(self.targets_dir, path_to_symlink)
        except OSError as e:
             # Replace the old symlink if an old symlink with the same
             # name exists.
            if e.errno == errno.EEXIST:
                os.remove(path_to_symlink)
                os.symlink(self.targets_dir, path_to_symlink)
            else:
                rospy.logerr('Could not create symlink to the '
                    + 'latest targets directory')

        # Client used to update the interop server.
        self.client = client

        # Highest file id so far.
        self.file_id = 0

        # Dictionary for storing Targets.
        # {file_id (int): target (Target)}
        self.targets = {}

    def load_all_remote_targets(self):
        """Loads all targets stored remotely to sync up state on startup."""
        remote_targets = self.client.get_all_targets()
        rospy.loginfo("Found %d remote targets", len(remote_targets))
        for target_id, target in remote_targets.iteritems():
            json_target = json.dumps(target)
            file_id = self.add_target(json_target, target_id)
            try:
                img = self.client.get_target_image(target_id)
                png = serializers.TargetImageSerializer.from_msg(img)
                self.set_target_image(file_id, png, False)
            except Exception as e:
                rospy.logerr("Could not get target %d image: %r", target_id, e)

    def clear_all_targets(self):
        """Clears all targets both remotely and locally."""
        # Deal with locally stored targets first.
        for target_id, target in self.targets.iteritems():
            target.delete()
            del self.targets[target_id]

        # Need to load targets stored remotely.
        remote_targets = self.client.get_all_targets()
        for target_id, target in remote_targets.iteritems():
            try:
                self.client.delete_target(target_id)
            except Exception as e:
                rospy.logerr("Could not delete remote target %d: %r",
                             target_id, e)

    def add_target(self, data, interop_id=None):
        """Adds a target.

        Args:
            data (str): The target data.
            interop_id (int): The associated ID on the server, optional.

        Returns:
            The file_id (int) of the added target.

        Raises:
            IOError: If the target could not be added.
        """
        with self.lock:
            # New file_id.
            file_id = self.file_id + 1

            target = Target(self.targets_dir, file_id, data,
                            self.client, interop_id)

            self.targets[file_id] = target
            # Record the largest file_id so far.
            self.file_id = file_id

        return file_id

    def update_target(self, file_id, data):
        """Updates an existing target.

        Args:
            file_id (int): The file id of the target file to update.
            data (str): The target data.

        Raises:
            KeyError: If the file_id does not exist in the self.targets
                dictionary.
            IOError: If the target could not be written.
        """
        with self.lock:
            target = self.targets[file_id]

        target.update(data)

    def delete_target(self, file_id):
        """Deletes an existing target.

        Args:
            file_id (int): The file id of the target to delete.

        Raises:
            KeyError: If the file_id does not exist in the self.targets
                dictionary.
            OSError: If the target file or the target image could not be deleted.
        """
        with self.lock:
            target = self.targets[file_id]

        target.delete()

    def get_target(self, file_id):
        """Returns a target as a str.

        Args:
            file_id (int): The file id of the target file to get.

        Returns:
            str: The contents of the target file

        Raises:
            KeyError: If the file_id does not exist in the self.targets
                dictionary.
            IOError: If the path to the target file is not known, or if the
                file could not be read.
        """
        with self.lock:
            target = self.targets[file_id]

        return target.get()

    def get_all_targets(self):
        """Returns all the targets as a dict.

        Returns:
            dict: The targets. {file_id (int): target (Target)}

        Raises:
            IOError: If the path to the one of the target files is not known,
                or if one of the files could not be read.
        """
        targets = {}

        with self.lock:
            for file_id, target in self.targets.iteritems():
                targets[file_id] = target.get()

        return targets

    def set_target_image(self, file_id, png_image, needs_adding=True):
        """Associates an image with a target or updates an existing target
        image.

        Args:
            file_id (int): The file id of the target to associate the image
                with.
            png_image (str): The image to add or update.
            needs_adding (bool): Whether we need to add the image to the
                server.

        Raises:
            KeyError: If the file_id does not exist in the self.targets
                dictionary.
            IOError: If the image could not be written.
        """
        with self.lock:
            target = self.targets[file_id]

        target.set_image(png_image, needs_adding)

    def delete_target_image(self, file_id):
        """Deletes an existing target image.

        Args:
            file_id (int): The file id of the target associated with the image.

        Raises:
            KeyError: If the file_id does not exist in the self.targets
                dictionary.
            OSError: If the target image could not be deleted.
        """
        with self.lock:
            target = self.targets[file_id]

        target.delete_image()

    def get_target_image(self, file_id):
        """Returns a target image as a str.

        Args:
            file_id (int): The file id of the target associated with the image.

        Returns:
            str: The target image.

        Raises:
            KeyError: If the file_id does not exist in the self.targets
                dictionary.
            IOError: If the path to the image file is not known, or if the
                file could not be read.
        """
        with self.lock:
            target = self.targets[file_id]

        return target.get_image()

    def sync(self):
        """Syncs all the targets and their images to the interop server."""
        with self.lock:
            # Sync all targets.
            for target_id, target in self.targets.iteritems():
                target.sync()

            # Delete unused targets from the targets dictionary.
            for file_id in list(self.targets):
                target = self.targets[file_id]
                if target.can_be_forgotten():
                    del self.targets[file_id]
