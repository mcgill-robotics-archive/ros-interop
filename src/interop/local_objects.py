# -*- coding: utf-8 -*-

import os
import json
import rospy
import os.path
import threading
import serializers
from cv_bridge import CvBridgeError
from requests.exceptions import ConnectionError, HTTPError, Timeout


class Object(object):

    """Represents an object and its image that is stored inside a local
    objects directory.
    Contains a sync function that syncs the object and its image to the
    interop server.
    """

    def __init__(self, objects_dir, file_id, data, client, interop_id=None):
        """Creates the object file with the specified data, inside the
        specified directory.

        Args:
            objects_dir (str): Absolute path to the parent directory
                of the object file.
            file_id (int): ID associated with this object.
            data (str): The object data that will be written into the object
                file.
            client (interop.InteroperabilityClient): Interoperability client
                that will be used to sync the object and its image to the
                server.
            interop_id (int): Remote ID associated with this object, optional.

        Raises:
            IOError: If the object file could not be written.
        """
        # Reentrant lock needed for sync where there are attempts to
        # acquire the lock twice.
        # Dual use: preventing simultaneous file IO, and preventing
        # simultaneous changes to the state variables:
        #   _needs_adding, _needs_updating, etc.
        self.lock = threading.RLock()

        self.client = client
        self.objects_dir = objects_dir

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
        # self.object_path may become None when object is deleted locally.
        self.object_path = os.path.join(self.objects_dir, filename)
        self.image_path = None

        # Create the object file.
        with self.lock:
            try:
                with open(self.object_path, "w", 0) as f:
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
            # No need to update object if it has not been added.
            if self._needs_adding:
                self._needs_updating = False
            # No need to update object if it is to be deleted.
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
            # If the object is not on the the server,
            # there is no need to do anything.
            if self.interop_id is None:
                self._needs_adding = False
                self._needs_updating = False
                self._needs_deleting = False
            # If the object is on the server, there is no need to
            # do anything other than deleting from the server.
            else:
                self._needs_adding = False
                self._needs_updating = False
                self._needs_deleting = True

            # No need to do anything with the image as it will be
            # deleted together with the object anyway.
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
        """Update this object.

        Args:
            data (str): The updated string to write into this object.

        Raises:
            IOError: If the path to the object file is not known,
                or if the object file could not be written.
        """
        with self.lock:
            if self.object_path is None:
                raise IOError("Could not update object for file_id {}. "
                              "Path to object file not known.".format(
                                  self.file_id))
            else:
                try:
                    with open(self.object_path, "w", 0) as f:
                        f.write(data)
                except IOError as e:
                    raise

                self.needs_updating = True

    def delete(self):
        """Delete this object and its associated image.
        Will silently fail if there is nothing to delete.

        Raises:
            OSError: If the path to the object file is not known,
                or if the object file or image could not be deleted.
        """
        with self.lock:
            # Delete object.
            if self.object_path is None:
                raise IOError("Could not delete object for file_id {}. "
                              "Path to object file not known.".format(
                                  self.file_id))
            else:
                try:
                    os.remove(self.object_path)
                except OSError as e:
                    raise

                self.object_path = None

            # Delete associated image.
            if self.image_path is not None:
                try:
                    os.remove(self.image_path)
                except OSError as e:
                    raise

                self.image_path = None

            self.needs_deleting = True

    def get(self):
        """Returns the content of the object file.

        Returns:
            str: The contents of the object file.

        Raises:
            IOError: If the path to the object file is not known, or if the
                file could not be read.
        """
        with self.lock:
            if self.object_path is None:
                raise IOError("Object file for file_id {} does not exist."
                              .format(self.file_id))
            else:
                try:
                    with open(self.object_path, "r") as f:
                        data = f.read()
                except IOError as e:
                    raise

                return data

    def set_image(self, png_image, needs_adding=True):
        """Associate an image with this object, or update the existing image.

        Args:
            png_image (str): The PNG image to be written.
            needs_adding (bool): Whether the image needs to be added to the
                server or not.

        Raises:
            IOError: If the image could not be written.
        """
        with self.lock:
            filename = str(self.file_id) + ".png"
            self.image_path = os.path.join(self.objects_dir, filename)

            try:
                with open(self.image_path, "wb", 0) as f:
                    f.write(png_image)
            except IOError as e:
                raise

            self.image_needs_setting = needs_adding

    def delete_image(self):
        """Delete the image associated with this object.
        Will silently fail if there is nothing to delete.

        Raises:
            OSError: If the path to the object image is not known,
                or if the object image could not be deleted.
        """
        with self.lock:
            if self.image_path is None:
                raise IOError("Could not delete image for file_id {}. "
                              "Path to image file not known.".format(
                                  self.file_id))
            else:
                try:
                    os.remove(self.image_path)
                except OSError as e:
                    raise

            self.image_needs_deleting = True

    def get_image(self):
        """Delete this object and its associated image.

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
        """Syncs this object and its image to the interop server."""
        with self.lock:
            # TARGET FILE
            if self.needs_adding:
                try:
                    object_ = self.get()
                except IOError as e:
                    rospy.logerr(e)
                else:
                    try:
                        # Post object and record the interop_id.
                        self.interop_id = self.client.post_object(object_)
                    except (ConnectionError, Timeout) as e:
                        rospy.logwarn(e)
                    except (ValueError, HTTPError) as e:
                        rospy.logerr(e)
                    else:
                        # No longer needs adding.
                        self.needs_adding = False

            # An interop id is needed to update.
            elif self.needs_updating and self.interop_id is not None:
                try:
                    object_ = self.get()
                except IOError as e:
                    rospy.logerr(e)
                else:
                    try:
                        self.client.put_object(self.interop_id, object_)
                    except (ConnectionError, Timeout) as e:
                        rospy.logwarn(e)
                    except (ValueError, HTTPError) as e:
                        rospy.logerr(e)
                    else:
                        self.needs_updating = False

            elif self.needs_deleting and self.interop_id is not None:
                try:
                    self.client.delete_object(self.interop_id)
                except (ConnectionError, Timeout) as e:
                    rospy.logwarn(e)
                except (ValueError, HTTPError) as e:
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
                        self.client.post_object_image(self.interop_id, image)
                    except (ConnectionError, Timeout) as e:
                        rospy.logwarn(e)
                    except (CvBridgeError, HTTPError) as e:
                        rospy.logerr(e)
                    else:
                        self.image_is_on_server = True
                        self.image_needs_setting = False

            elif (self.image_needs_deleting and self.image_is_on_server and
                  self.interop_id is not None):
                try:
                    self.client.delete_object_image(self.interop_id)
                except (ConnectionError, Timeout) as e:
                    rospy.logwarn(e)
                except (CvBridgeError, HTTPError) as e:
                    rospy.logerr(e)
                else:
                    self.image_is_on_server = False
                    self.image_needs_deleting = False

    def can_be_forgotten(self):
        """When an object is removed locally and on the interop server, it
        no longer has any use. References to an object may still exist elsewhere
        in program. This method helps to determine if those references can be
        deleted.

        Returns:
            True if the object object can be forgotten, False otherwise.
        """
        with self.lock:
            # If the object and/or its image is still stored locally.
            if self.object_path or self.image_path:
                return False

            # If there are still things to be done on the interop server.
            if (self.needs_adding or self.needs_updating or
                    self.needs_deleting or self.image_needs_setting or
                    self.image_needs_deleting):
                return False

            # Otherwise, the object is useless and all references can be
            # deleted.
            return True


class ObjectsDirectory(object):

    """Represents the directory containing object files and images.
    Contains a sync function that syncs the contents of the directory to
    the interop server.
    """

    def __init__(self, path, client):
        """Creates a directory for storing objects and images.

        Args:
            path (str): Absolute path to the directory of to be created.
            client (interop.InteroperabilityClient): Interoperability client
                that will be used to handle syncs to the interop server.
        """
        self.lock = threading.Lock()
        self.path = path

        # Client used to update the interop server.
        self.client = client

        # Highest file id so far.
        self.file_id = 0

        # Dictionary for storing Objects.
        # {file_id (int): object (Object)}
        self.objects = {}

    def load_all_remote_objects(self):
        """Loads all objects stored remotely to sync up state on startup."""
        remote_objects = self.client.get_all_objects()
        rospy.loginfo("Found %d remote objects", len(remote_objects))
        for object_id, object_ in remote_objects.iteritems():
            json_object = json.dumps(object_)
            file_id = self.add_object(json_object, object_id)
            try:
                img = self.client.get_object_image(object_id)
                png = serializers.ObjectImageSerializer.from_msg(img)
                self.set_object_image(file_id, png, False)
            except Exception as e:
                rospy.logerr("Could not get object %d image: %r", object_id, e)

    def clear_all_objects(self):
        """Clears all objects both remotely and locally."""
        # Deal with locally stored objects first.
        for object_id, object_ in self.objects.iteritems():
            object_.delete()
        self.objects.clear()

        # Need to load objects stored remotely.
        remote_objects = self.client.get_all_objects()
        for object_id, object_ in remote_objects.iteritems():
            try:
                self.client.delete_object(object_id)
            except Exception as e:
                rospy.logerr("Could not delete remote object %d: %r", object_id,
                             e)

    def add_object(self, data, interop_id=None):
        """Adds an object.

        Args:
            data (str): The object data.
            interop_id (int): The associated ID on the server, optional.

        Returns:
            The file_id (int) of the added object.

        Raises:
            IOError: If the object could not be added.
        """
        with self.lock:
            # New file_id.
            file_id = self.file_id + 1

            object_ = Object(self.path, file_id, data, self.client, interop_id)

            self.objects[file_id] = object_
            # Record the largest file_id so far.
            self.file_id = file_id

        return file_id

    def update_object(self, file_id, data):
        """Updates an existing object.

        Args:
            file_id (int): The file id of the object file to update.
            data (str): The object data.

        Raises:
            KeyError: If the file_id does not exist in the self.objects
                dictionary.
            IOError: If the object could not be written.
        """
        with self.lock:
            object_ = self.objects[file_id]

        object_.update(data)

    def delete_object(self, file_id):
        """Deletes an existing object.

        Args:
            file_id (int): The file id of the object to delete.

        Raises:
            KeyError: If the file_id does not exist in the self.objects
                dictionary.
            OSError: If the object file or the object image could not be
                deleted.
        """
        with self.lock:
            object_ = self.objects[file_id]

        object_.delete()

    def get_object(self, file_id):
        """Returns an object as a str.

        Args:
            file_id (int): The file id of the object file to get.

        Returns:
            str: The contents of the object file

        Raises:
            KeyError: If the file_id does not exist in the self.objects
                dictionary.
            IOError: If the path to the object file is not known, or if the
                file could not be read.
        """
        with self.lock:
            object_ = self.objects[file_id]

        return object_.get()

    def get_all_objects(self):
        """Returns all the objects as a dict.

        Returns:
            dict: The objects. {file_id (int): object (Object)}

        Raises:
            IOError: If the path to the one of the object files is not known,
                or if one of the files could not be read.
        """
        objects = {}

        with self.lock:
            for file_id, object_ in self.objects.iteritems():
                objects[file_id] = object_.get()

        return objects

    def set_object_image(self, file_id, png_image, needs_adding=True):
        """Associates an image with an object or updates an existing object
        image.

        Args:
            file_id (int): The file id of the object to associate the image
                with.
            png_image (str): The image to add or update.
            needs_adding (bool): Whether we need to add the image to the
                server.

        Raises:
            KeyError: If the file_id does not exist in the self.objects
                dictionary.
            IOError: If the image could not be written.
        """
        with self.lock:
            object_ = self.objects[file_id]

        object_.set_image(png_image, needs_adding)

    def delete_object_image(self, file_id):
        """Deletes an existing object image.

        Args:
            file_id (int): The file id of the object associated with the image.

        Raises:
            KeyError: If the file_id does not exist in the self.objects
                dictionary.
            OSError: If the object image could not be deleted.
        """
        with self.lock:
            object_ = self.objects[file_id]

        object_.delete_image()

    def get_object_image(self, file_id):
        """Returns an object image as a str.

        Args:
            file_id (int): The file id of the object associated with the image.

        Returns:
            str: The object image.

        Raises:
            KeyError: If the file_id does not exist in the self.objects
                dictionary.
            IOError: If the path to the image file is not known, or if the
                file could not be read.
        """
        with self.lock:
            object_ = self.objects[file_id]

        return object_.get_image()

    def sync(self):
        """Syncs all the objects and their images to the interop server."""
        with self.lock:
            # Sync all objects.
            for object_id, object_ in self.objects.iteritems():
                object_.sync()

            # Delete unused objects from the objects dictionary.
            for file_id in list(self.objects):
                object_ = self.objects[file_id]
                if object_.can_be_forgotten():
                    del self.objects[file_id]
