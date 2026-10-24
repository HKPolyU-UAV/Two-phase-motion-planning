#!/usr/bin/env python3
"""Mirror the planner's dynamic obstacle markers into Gazebo Classic.

The planner publishes complete obstacle snapshots on ``obs``. Poses, dimensions
and colors come from those snapshots, so this node never integrates motion a
second time. Explicit DELETE / DELETEALL arrays are also accepted as updates.
"""

import hashlib
import math
import re
import threading
import time
import xml.etree.ElementTree as ET

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import (DeleteModel, GetModelProperties, GetWorldProperties,
                             SetModelState, SpawnModel)
from visualization_msgs.msg import Marker, MarkerArray


MARKER_NAMESPACE = re.compile(r"dyn_obs[1-9][0-9]*$")
MODEL_LINK = "rviz_dynamic_obstacle_link"
MODEL_COLLISION = "rviz_dynamic_obstacle_collision_"


def geometry_signature(marker):
    return (marker.scale.x, marker.scale.y, marker.scale.z,
            marker.color.r, marker.color.g, marker.color.b, marker.color.a)


def geometry_key(marker):
    values = " ".join(format(value, ".17g") for value in geometry_signature(marker))
    return hashlib.sha256(values.encode("ascii")).hexdigest()[:16]


def cylinder_sdf(name, marker):
    """A prescribed moving obstacle: collision is enabled, free drift is not."""
    sdf = ET.Element("sdf", version="1.6")
    model = ET.SubElement(sdf, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name=MODEL_LINK)
    ET.SubElement(link, "gravity").text = "false"
    for kind in ("collision", "visual"):
        # The collision name preserves the geometry/color signature in Gazebo,
        # allowing another bridge process to adopt it without delete/respawn.
        element_name = MODEL_COLLISION + geometry_key(marker) if kind == "collision" else kind
        element = ET.SubElement(link, kind, name=element_name)
        cylinder = ET.SubElement(ET.SubElement(element, "geometry"), "cylinder")
        ET.SubElement(cylinder, "radius").text = str(marker.scale.x / 2.0)
        ET.SubElement(cylinder, "length").text = str(marker.scale.z)
        if kind == "visual":
            material = ET.SubElement(element, "material")
            color = "{} {} {} 1".format(marker.color.r, marker.color.g, marker.color.b)
            ET.SubElement(material, "ambient").text = color
            ET.SubElement(material, "diffuse").text = color
            ET.SubElement(element, "transparency").text = str(1.0 - marker.color.a)
    return ET.tostring(sdf, encoding="unicode")


class GazeboDynamicObstacles:
    def __init__(self):
        namespace = rospy.get_namespace().strip("/").replace("/", "_")
        self.prefix = rospy.get_param("~model_prefix", (namespace + "_") if namespace else "")
        if not re.fullmatch(r"[A-Za-z0-9_]*", self.prefix):
            raise ValueError("model_prefix must contain only letters, digits and underscores")
        self.rate = float(rospy.get_param("~update_rate", 10.0))
        if not math.isfinite(self.rate) or self.rate <= 0:
            raise ValueError("update_rate must be positive and finite")
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._desired = {}
        self._have_input = False
        self._known = set()
        self._owned = set()
        self._geometry = {}
        self._last_refresh = float("-inf")
        self._world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        self._properties = rospy.ServiceProxy("/gazebo/get_model_properties", GetModelProperties)
        self._spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self._delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self._move = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self._subscriber = rospy.Subscriber("obs", MarkerArray, self._on_markers, queue_size=1)
        rospy.on_shutdown(self._stop.set)

    def _valid(self, marker):
        p, q = marker.pose.position, marker.pose.orientation
        values = geometry_signature(marker) + (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        return (marker.type == Marker.CYLINDER and marker.header.frame_id.lstrip("/") == "world"
                and all(math.isfinite(value) for value in values)
                and min(marker.scale.x, marker.scale.y, marker.scale.z) > 0
                and math.isclose(marker.scale.x, marker.scale.y, rel_tol=1e-6)
                and all(0 <= value <= 1 for value in values[3:7])
                and q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w > 0)

    def _on_markers(self, message):
        # ADD-only messages from MotionPrimitives are full snapshots. Deletion
        # messages keep the standard Marker semantics and affect only their keys.
        with self._lock:
            has_delete = any(m.action in (Marker.DELETE, Marker.DELETEALL) for m in message.markers)
            desired = dict(self._desired) if has_delete else {}
            for marker in message.markers:
                if marker.action == Marker.DELETEALL:
                    desired.clear()
                elif MARKER_NAMESPACE.fullmatch(marker.ns):
                    name = self.prefix + marker.ns
                    if marker.action == Marker.DELETE:
                        desired.pop(name, None)
                    elif marker.action == Marker.ADD:
                        if self._valid(marker):
                            desired[name] = marker
                        else:
                            rospy.logwarn_throttle(5.0, "Ignoring invalid dynamic cylinder marker %s", marker.ns)
            self._desired = desired
            self._have_input = True

    def _refresh_models(self):
        response = self._world()
        if not response.success:
            raise rospy.ServiceException(response.status_message)
        self._known = set(response.model_names)
        self._owned.intersection_update(self._known)
        self._geometry = {n: g for n, g in self._geometry.items() if n in self._known}
        # Discover only models produced by this bridge. A colliding name alone
        # is insufficient authority to move or delete another model.
        for name in self._known - self._owned:
            if not name.startswith(self.prefix) or not MARKER_NAMESPACE.fullmatch(name[len(self.prefix):]):
                continue
            properties = self._properties(name)
            if properties.success and [n.split("::")[-1] for n in properties.body_names] == [MODEL_LINK]:
                self._owned.add(name)
                if len(properties.geom_names) == 1:
                    collision_name = properties.geom_names[0].split("::")[-1]
                    match = re.fullmatch(re.escape(MODEL_COLLISION) + r"([0-9a-f]{16})", collision_name)
                    if match:
                        self._geometry[name] = match.group(1)
        self._last_refresh = time.monotonic()

    def _remove(self, name):
        if name in self._known:
            response = self._delete(name)
            if not response.success:
                raise rospy.ServiceException(response.status_message)
        self._known.discard(name)
        self._owned.discard(name)
        self._geometry.pop(name, None)

    def _sync(self):
        if time.monotonic() - self._last_refresh >= 1.0:
            self._refresh_models()
        with self._lock:
            if not self._have_input:
                return
            desired = dict(self._desired)
        for name in self._owned - desired.keys():
            self._remove(name)
        for name, marker in desired.items():
            if name in self._known and name not in self._owned:
                rospy.logwarn_throttle(5.0, "Gazebo model %s is not owned by the obstacle bridge; leaving it untouched", name)
                continue
            signature = geometry_key(marker)
            # Recreate only changed geometry, or legacy bridge models lacking
            # the signature. Unchanged models survive bridge restarts in place.
            if name in self._owned and self._geometry.get(name) != signature:
                self._remove(name)
            if name not in self._known:
                response = self._spawn(name, cylinder_sdf(name, marker), rospy.get_namespace(),
                                       marker.pose, "world")
                if not response.success:
                    raise rospy.ServiceException(response.status_message)
                self._known.add(name)
                self._owned.add(name)
                self._geometry[name] = signature
                rospy.loginfo("Created Gazebo dynamic obstacle %s", name)
            state = ModelState(model_name=name, pose=marker.pose, reference_frame="world")
            # Zero twist is intentional: only marker updates move these models.
            # Reapplying poses also restores them after Gazebo reset_world.
            response = self._move(state)
            if not response.success:
                self._last_refresh = float("-inf")
                raise rospy.ServiceException(response.status_message)

    def run(self):
        # Wall time keeps service recovery and deletion working while Gazebo is
        # paused; the desired pose itself advances only with planner markers.
        while not rospy.is_shutdown():
            started = time.monotonic()
            try:
                self._sync()
            except (rospy.ServiceException, rospy.ROSException) as error:
                self._last_refresh = float("-inf")
                rospy.logwarn_throttle(5.0, "Waiting to synchronize Gazebo obstacles: %s", error)
                if self._stop.wait(1.0):
                    break
            if self._stop.wait(max(0.0, 1.0 / self.rate - (time.monotonic() - started))):
                break


if __name__ == "__main__":
    rospy.init_node("gazebo_dynamic_obstacles")
    try:
        GazeboDynamicObstacles().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
