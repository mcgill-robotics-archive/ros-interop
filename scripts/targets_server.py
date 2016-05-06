#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Interoperability Target ROS Server."""

import rospy
import interop.srv
from cv_bridge import CvBridgeError
from simplejson import JSONDecodeError
from interop import InteroperabilityClient
from requests.exceptions import HTTPError, Timeout


def add_target(req):
    """Handles AddTarget service request.

    Args:
        req: AddTargetRequest message.

    Returns:
        AddTargetResponse.
    """
    response = interop.srv.AddTargetResponse()
    response.success = False

    try:
        response.id = client.post_target(req.target)
        response.success = True
    except Timeout as e:
        rospy.logwarn(e)
    except (JSONDecodeError, HTTPError) as e:
        rospy.logerr(e)

    return response


def get_target(req):
    """Handles GetTarget service request.

    Args:
        req: GetTargetRequest message.

    Returns:
        GetTargetResponse.
    """
    response = interop.srv.GetTargetResponse()
    response.success = False

    try:
        response.target = client.get_target(req.id)
        response.success = True
    except Timeout as e:
        rospy.logwarn(e)
    except (JSONDecodeError, HTTPError) as e:
        rospy.logerr(e)

    return response


def update_target(req):
    """Handles UpdateTarget service request.

    Args:
        req: UpdateTargetRequest message.

    Returns:
        UpdateTargetResponse.
    """
    response = interop.srv.UpdateTargetResponse()
    response.success = False

    try:
        client.put_target(req.id, req.target)
        response.success = True
    except Timeout as e:
        rospy.logwarn(e)
    except HTTPError as e:
        rospy.logerr(e)

    return response


def delete_target(req):
    """Handles DeleteTarget service request.

    Args:
        req: DeleteTargetRequest message.

    Returns:
        DeleteTargetResponse.
    """
    response = interop.srv.DeleteTargetResponse()
    response.success = False

    try:
        client.delete_target(req.id)
        response.success = True
    except Timeout as e:
        rospy.logwarn(e)
    except HTTPError as e:
        rospy.logerr(e)

    return response


def get_all_targets(req):
    """Handles GetAllTargets service request.

    Args:
        req: GetAllTargetsRequest message.

    Returns:
        GetAllTargetsResponse.
    """
    response = interop.srv.GetAllTargetsResponse()
    response.success = False

    try:
        for id, target in client.get_all_targets().iteritems():
            response.ids.append(id)
            response.targets.append(target)

        response.success = True
    except Timeout as e:
        rospy.logwarn(e)
    except (JSONDecodeError, HTTPError) as e:
        rospy.logerr(e)

    return response


def add_target_image(req):
    """Handles AddTargetImage service request.

    Args:
        req: AddTargetImageRequest message.

    Returns:
        AddTargetImageResponse.
    """
    response = interop.srv.AddTargetImageResponse()
    response.success = False

    try:
        client.post_target_image(req.id, req.image)
        response.success = True
    except Timeout as e:
        rospy.logwarn(e)
    except (CvBridgeError, HTTPError) as e:
        rospy.logerr(e)

    return response


def get_target_image(req):
    """Handles GetTargetImage service request.

    Args:
        req: GetTargetImageRequest message.

    Returns:
        GetTargetImageResponse.
    """
    response = interop.srv.GetTargetImageResponse()
    response.success = False

    try:
        response.image = client.get_target_image(req.id)
        response.success = True
    except Timeout as e:
        rospy.logwarn(e)
    except (CvBridgeError, HTTPError) as e:
        rospy.logerr(e)

    return response


def delete_target_image(req):
    """Handles DeleteTargetImage service request.

    Args:
        req: DeleteTargetImageRequest message.

    Returns:
        DeleteTargetImageResponse.
    """
    response = interop.srv.DeleteTargetImageResponse()
    response.success = False

    try:
        client.delete_target_image(req.id)
        response.success = True
    except Timeout as e:
        rospy.logwarn(e)
    except HTTPError as e:
        rospy.logerr(e)

    return response


if __name__ == "__main__":
    # Initialize node.
    rospy.init_node("targets")

    # Get ROS parameters for client.
    base_url = rospy.get_param("~base_url")
    username = rospy.get_param("~username")
    password = rospy.get_param("~password")
    timeout = rospy.get_param("~timeout")

    # Initialize interoperability client.
    client = InteroperabilityClient(base_url, username, password, timeout)

    # Initialize target ROS services.
    rospy.Service("~add", interop.srv.AddTarget, add_target)
    rospy.Service("~get", interop.srv.GetTarget, get_target)
    rospy.Service("~update", interop.srv.UpdateTarget, update_target)
    rospy.Service("~delete", interop.srv.DeleteTarget, delete_target)
    rospy.Service("~all", interop.srv.GetAllTargets, get_all_targets)

    # Initialize target image ROS services.
    rospy.Service("~image/add", interop.srv.AddTargetImage, add_target_image)
    rospy.Service("~image/get", interop.srv.GetTargetImage, get_target_image)
    rospy.Service("~image/delete", interop.srv.DeleteTargetImage,
                  delete_target_image)

    # Spin forever.
    rospy.spin()
