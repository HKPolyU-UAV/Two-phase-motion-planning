#!/usr/bin/env python3
"""Clip the visualization-only primitive overlay away from the front-camera lens.

The original planner markers and commands are untouched. Points remain in their
original marker frame so RViz projects them into the calibrated camera view.
"""

import copy
import math

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def clip_polyline(vertices, camera_from_marker, minimum_depth):
    """Return separate visible runs, interpolating crossings of optical Z=near.

    Vertices have XYZ in the first three columns and may carry extra interpolated
    attributes (RGBA). Invalid vertices break a run instead of joining across it.
    """
    vertices = np.asarray(vertices, dtype=float)
    if vertices.size == 0:
        return []
    if vertices.ndim != 2 or vertices.shape[1] < 3:
        raise ValueError("expected an N x 3 (or wider) vertex array")
    transform = np.asarray(camera_from_marker, dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("camera transform must be a finite 4 x 4 matrix")
    if not math.isfinite(minimum_depth) or minimum_depth <= 0:
        raise ValueError("minimum_depth must be positive and finite")
    valid = np.isfinite(vertices).all(axis=1)
    depth = vertices[:, :3] @ transform[2, :3] + transform[2, 3]
    visible = np.flatnonzero(valid & (depth >= minimum_depth))
    if not visible.size:
        return []
    groups = np.split(visible, np.flatnonzero(np.diff(visible) > 1) + 1)
    result = []
    for group in groups:
        first, last = int(group[0]), int(group[-1])
        pieces = []
        if first > 0 and valid[first - 1]:
            a, b = first - 1, first
            weight = (minimum_depth - depth[a]) / (depth[b] - depth[a])
            pieces.append((vertices[a] + weight * (vertices[b] - vertices[a]))[None, :])
        pieces.append(vertices[first:last + 1])
        if last + 1 < len(vertices) and valid[last + 1]:
            a, b = last, last + 1
            weight = (minimum_depth - depth[a]) / (depth[b] - depth[a])
            pieces.append((vertices[a] + weight * (vertices[b] - vertices[a]))[None, :])
        run = np.concatenate(pieces, axis=0)
        # A path that merely touches the plane is not a drawable line segment.
        keep = np.r_[True, np.linalg.norm(np.diff(run[:, :3], axis=0), axis=1) > 1e-9]
        run = run[keep]
        if len(run) >= 2:
            result.append(run)
    return result


def pose_matrix(pose):
    quaternion = (pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w)
    matrix = tf.transformations.quaternion_matrix(quaternion)
    matrix[:3, 3] = (pose.position.x, pose.position.y, pose.position.z)
    if not np.isfinite(matrix).all():
        raise ValueError("marker pose is not finite")
    return matrix


class FrontCameraPrimitives:
    def __init__(self):
        self.camera_frame = rospy.get_param("~camera_frame", "suav1/front_camera_optical_frame").lstrip("/")
        self.minimum_depth = float(rospy.get_param("~minimum_depth", 5.0))
        self.candidate_width = float(rospy.get_param("~candidate_width", 0.03))
        self.optimal_width = float(rospy.get_param("~optimal_width", 0.06))
        if not self.camera_frame or any(not math.isfinite(v) or v <= 0 for v in
                (self.minimum_depth, self.candidate_width, self.optimal_width)):
            raise ValueError("camera frame and positive finite overlay dimensions are required")
        self.listener = tf.TransformListener()
        self.publisher = rospy.Publisher("front_camera/primitives", MarkerArray, queue_size=1)
        self.subscriber = rospy.Subscriber("traj", MarkerArray, self.receive,
                                          queue_size=1, buff_size=8 * 1024 * 1024)

    def receive(self, message):
        clear = Marker()
        # Clearing must not itself depend on the missing camera transform.
        clear.header.frame_id = next((m.header.frame_id.lstrip("/") for m in message.markers
                                      if m.header.frame_id), "world")
        clear.action = Marker.DELETEALL
        output = MarkerArray(markers=[clear])
        transforms = {}
        for marker in message.markers:
            if marker.action != Marker.ADD or marker.type != Marker.LINE_STRIP or len(marker.points) < 2:
                continue
            frame = marker.header.frame_id.lstrip("/")
            if frame not in transforms:
                try:
                    translation, rotation = self.listener.lookupTransform(self.camera_frame, frame, rospy.Time(0))
                    transform = tf.transformations.quaternion_matrix(rotation)
                    transform[:3, 3] = translation
                    transforms[frame] = transform
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as error:
                    transforms[frame] = None
                    rospy.logwarn_throttle(5.0, "Front-camera overlay waiting for TF: %s", error)
            if transforms[frame] is None:
                continue
            vertices = np.array([(p.x, p.y, p.z) for p in marker.points], dtype=float)
            per_vertex_color = len(marker.colors) == len(marker.points)
            if per_vertex_color:
                colors = np.array([(c.r, c.g, c.b, c.a) for c in marker.colors], dtype=float)
                vertices = np.column_stack((vertices, colors))
            try:
                runs = clip_polyline(vertices, transforms[frame] @ pose_matrix(marker.pose), self.minimum_depth)
            except ValueError as error:
                rospy.logwarn_throttle(5.0, "Skipping invalid front-camera primitive: %s", error)
                continue
            for run in runs:
                clipped = Marker()
                clipped.header = copy.copy(marker.header)
                clipped.ns = "front_camera/" + marker.ns
                clipped.id = len(output.markers)
                clipped.type, clipped.action = Marker.LINE_STRIP, Marker.ADD
                clipped.pose = copy.deepcopy(marker.pose)
                clipped.scale.x = self.optimal_width if marker.ns == "traj_optimal" else self.candidate_width
                clipped.color = copy.copy(marker.color)
                clipped.lifetime = marker.lifetime
                clipped.frame_locked = marker.frame_locked
                clipped.points = [Point(*row[:3]) for row in run]
                if per_vertex_color:
                    clipped.colors = [ColorRGBA(*row[3:7]) for row in run]
                output.markers.append(clipped)
        # Clearing first also removes old segments after a path shrinks or TF fails.
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("front_camera_primitives")
    FrontCameraPrimitives()
    rospy.spin()
