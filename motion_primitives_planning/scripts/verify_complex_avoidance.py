#!/usr/bin/env python3
"""Observe the first traversal of the complex 3-D avoidance course.

This node never commands the aircraft or Gazebo. Cylinder clearance uses the
odometry position as a point; it is evidence for this simulation, not a full-body
collision certificate. Dynamic centers are time-aligned from obstacle markers
and their configured motion. The private `passed` and `summary` parameters are
published when verification ends; an optional `report_file` saves the same JSON.
"""

import json
import math
import threading
from pathlib import Path

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("all configuration coordinates must be finite")
    return value


def cylinder_clearance(x, y, z, ox, oy, oz, radius, height):
    """Signed cylinder-separation metric: negative iff point is inside."""
    return max(math.hypot(x - ox, y - oy) - radius,
               abs(z - oz) - height / 2.0)


class Passage:
    def __init__(self, config, radius, half_width):
        self.name = str(config["name"])
        self.x = finite(config["x"])
        self.direction = config["direction"]
        if self.direction not in ("left", "right", "above", "below"):
            raise ValueError("unknown passage direction: " + str(self.direction))
        self.threshold = finite(config["threshold"])
        self.goal_height = finite(config["goal_height"])
        self.goal_y = finite(config.get("goal_y", 0.0))
        self.radius, self.half_width = radius, half_width
        self.samples = 0
        self.min_y = self.min_height = self.min_clearance = math.inf
        self.max_y = self.max_height = -math.inf
        self.saw_before = False
        self.previous = None
        self.crossing = None
        self.continuous = self.goal_verified = True
        self.complete = self.passed = False
        self.reason = "not yet crossed"

    def observe(self, x, y, z, goal, goal_fresh):
        if self.complete:
            return False
        front, rear = self.x - self.radius, self.x + self.radius
        if x < front:
            self.saw_before = True
        if front <= x <= rear:
            self.samples += 1
            self.min_y, self.max_y = min(self.min_y, y), max(self.max_y, y)
            self.min_height, self.max_height = min(self.min_height, z), max(self.max_height, z)
            if self.direction in ("above", "below"):
                clearance = z - self.threshold if self.direction == "above" else self.threshold - z
                self.min_clearance = min(self.min_clearance, clearance)
            self.goal_verified = (self.goal_verified and goal_fresh
                                  and abs(goal[1] - self.goal_y) <= 0.05
                                  and abs(goal[2] - self.goal_height) <= 0.05)
        if self.previous is not None:
            previous_x, previous_y, previous_z = self.previous
            if previous_x <= rear and x >= front and abs(x - previous_x) > 2.0:
                self.continuous = False
            if self.crossing is None and previous_x < self.x <= x:
                fraction = (self.x - previous_x) / (x - previous_x)
                crossing_y = previous_y + fraction * (y - previous_y)
                crossing_z = previous_z + fraction * (z - previous_z)
                self.crossing = (crossing_y, crossing_z)
                # At the cylinder row center the required lateral gap is exact.
                # At its rounded shoulders a smaller y can still be collision-free.
                if self.direction in ("left", "right"):
                    self.min_clearance = (crossing_y - self.threshold if self.direction == "left"
                                          else self.threshold - crossing_y)
        self.previous = (x, y, z)
        if x <= rear:
            return False
        self.complete = True
        problems = []
        if not self.saw_before or self.samples < 3 or not self.continuous or self.crossing is None:
            problems.append("insufficient continuous passage observations")
        if not self.goal_verified:
            problems.append("goal changed or was unavailable during passage")
        if self.min_clearance <= 0:
            problems.append("required " + self.direction + " passage threshold violated")
        if self.samples and max(abs(self.min_y), abs(self.max_y)) >= self.half_width:
            problems.append("bypassed the corridor boundary")
        self.passed = not problems
        self.reason = "; ".join(problems) if problems else "required passage verified while goal remained unchanged"
        return True

    def summary(self):
        return {
            "name": self.name, "x": self.x, "direction": self.direction,
            "threshold": self.threshold, "goal_y": self.goal_y, "goal_height": self.goal_height,
            "complete": self.complete, "passed": self.passed, "samples": self.samples,
            "goal_verified": self.goal_verified and self.samples > 0,
            "min_y": self.min_y if self.samples else 0.0,
            "max_y": self.max_y if self.samples else 0.0,
            "min_height": self.min_height if self.samples else 0.0,
            "max_height": self.max_height if self.samples else 0.0,
            "min_clearance": self.min_clearance if math.isfinite(self.min_clearance) else 0.0,
            "center_crossing_observed": self.crossing is not None,
            "crossing_y": self.crossing[0] if self.crossing else 0.0,
            "crossing_height": self.crossing[1] if self.crossing else 0.0,
            "reason": self.reason,
        }


class MovingCylinder:
    def __init__(self, name, config, default_height):
        self.name = name
        self.kind = config.get("type", "linear")
        if self.kind not in ("linear", "circular"):
            raise ValueError("unsupported dynamic motion: " + str(self.kind))
        self.radius = finite(config.get("r", 5.0))
        self.height = finite(config.get("height", default_height))
        self.z = finite(config.get("z", self.height / 2.0))
        self.center = tuple(finite(config.get("center", {}).get(k, 0.0)) for k in ("x", "y"))
        self.orbit = finite(config.get("radius", 0.0))
        self.omega = finite(config.get("angular_velocity", 0.0))
        self.velocity = finite(config.get("velocity", 0.0))
        self.direction = math.radians(finite(config.get("direction", 0.0)))
        if min(self.radius, self.height) <= 0 or self.orbit < 0:
            raise ValueError("dynamic cylinder dimensions must be positive")
        self.marker = None
        self.origin = None
        self.max_motion = 0.0
        self.samples = self.checked_samples = self.encounter_samples = 0
        self.blocked_reference_samples = 0
        self.min_horizontal = self.min_physical = self.min_inflated = math.inf
        self.nearest = []

    def update(self, marker):
        p = marker.pose.position
        sample = (marker.header.stamp.to_sec(), p.x, p.y, p.z)
        if (marker.action != Marker.ADD or marker.type != Marker.CYLINDER
                or marker.header.frame_id.lstrip("/") != "world"
                or not all(math.isfinite(v) for v in sample)):
            return
        if self.origin is None:
            self.origin = sample[1:3]
        self.max_motion = max(self.max_motion, math.hypot(p.x - self.origin[0], p.y - self.origin[1]))
        self.marker = sample
        self.samples += 1

    def center_at(self, timestamp):
        if self.marker is None or abs(timestamp - self.marker[0]) > 1.0:
            return None
        dt = timestamp - self.marker[0]
        _, x, y, z = self.marker
        if self.kind == "circular":
            angle = math.atan2(y - self.center[1], x - self.center[0]) + self.omega * dt
            x = self.center[0] + self.orbit * math.cos(angle)
            y = self.center[1] + self.orbit * math.sin(angle)
        else:
            x += self.velocity * math.cos(self.direction) * dt
            y += self.velocity * math.sin(self.direction) * dt
        return x, y, z

    def inspect(self, timestamp, x, y, z, goal, goal_fresh, encounter_distance):
        center = self.center_at(timestamp)
        if center is None:
            return None
        ox, oy, oz = center
        horizontal = math.hypot(x - ox, y - oy)
        physical = cylinder_clearance(x, y, z, ox, oy, oz, self.radius, self.height)
        inflated = cylinder_clearance(x, y, z, ox, oy, oz, 1.2 * self.radius, self.height + 2.0)
        self.checked_samples += 1
        self.min_physical, self.min_inflated = min(self.min_physical, physical), min(self.min_inflated, inflated)
        if horizontal < self.min_horizontal:
            self.min_horizontal = horizontal
            self.nearest = [timestamp, x, y, z, ox, oy, oz]
        vertical_gap = abs(z - oz) - self.height / 2.0
        if ((horizontal < encounter_distance and vertical_gap <= 0)
                or (horizontal < self.radius + 15.0 and vertical_gap <= 10.0)):
            self.encounter_samples += 1
        if goal_fresh and cylinder_clearance(x, goal[1], goal[2], ox, oy, oz,
                                             1.2 * self.radius, self.height + 2.0) <= 0:
            self.blocked_reference_samples += 1
        return physical, inflated

    def summary(self):
        return {
            "name": self.name, "marker_samples": self.samples,
            "checked_odometry_samples": self.checked_samples,
            "max_observed_motion": self.max_motion,
            "close_encounter_samples": self.encounter_samples,
            "blocked_goal_line_samples": self.blocked_reference_samples,
            "min_horizontal_center_distance": self.min_horizontal if self.checked_samples else 0.0,
            "min_physical_clearance": self.min_physical if self.checked_samples else 0.0,
            "min_inflated_clearance": self.min_inflated if self.checked_samples else 0.0,
            "nearest_sample_time_aircraft_xyz_obstacle_xyz": self.nearest,
        }


class ComplexAvoidanceChecker:
    def __init__(self):
        self.lock = threading.RLock()
        cfg = rospy.get_param("complex_validation")
        radius, half_width = finite(cfg.get("radius", 7.2)), finite(cfg.get("half_width", 80.0))
        self.final_x = finite(cfg.get("final_x", 860.0))
        self.final_height = finite(cfg.get("final_height", 8.0))
        self.final_tolerance = finite(cfg.get("final_tolerance", 3.0))
        self.timeout = finite(cfg.get("timeout", 120.0))
        self.motion_min = finite(cfg.get("dynamic_motion_min", 2.0))
        self.encounter_distance = finite(cfg.get("dynamic_encounter_distance", 20.0))
        if min(radius, half_width, self.final_tolerance, self.timeout, self.motion_min, self.encounter_distance) <= 0:
            raise ValueError("dimensions, tolerances and timeout must be positive")
        self.gates = [Passage(g, radius, half_width) for g in cfg["gates"]]
        if not self.gates or len({g.name for g in self.gates}) != len(self.gates):
            raise ValueError("gates must be nonempty and uniquely named")
        static = rospy.get_param("static_obstacles")
        self.static = []
        for i in range(1, int(static["num"]) + 1):
            obs = static["obs" + str(i)]
            h = finite(obs.get("height", static.get("height", 36.2743)))
            values = tuple(finite(v) for v in (obs["x"], obs["y"], obs.get("z", h / 2), obs["r"], h))
            if min(values[3:]) <= 0:
                raise ValueError("static cylinder dimensions must be positive")
            self.static.append(("static_obs" + str(i), values))
        dynamic = rospy.get_param("dynamic_obstacles")
        self.dynamic = {"dyn_obs" + str(i): MovingCylinder("dyn_obs" + str(i), dynamic["dyn_obs" + str(i)], dynamic.get("height", 10.0))
                        for i in range(1, int(dynamic["num"]) + 1)}
        if not self.dynamic:
            raise ValueError("complex validation requires dynamic obstacles")
        self.goal = (0.0, 0.0, 0.0)
        self.goal_time = self.start_time = self.last_time = None
        self.last_position = []
        self.min_static_physical = self.min_static_inflated = math.inf
        self.static_samples = 0
        self.dynamic_missing_samples = 0
        self.finished = False
        self.report_file = rospy.get_param("~report_file", "")
        if rospy.has_param("~passed"):
            rospy.delete_param("~passed")
        rospy.set_param("~summary", {"status": "waiting", "passed": False})
        self.subscribers = [rospy.Subscriber("odom", Odometry, self.on_odom, queue_size=50),
                            rospy.Subscriber("goal", Marker, self.on_goal, queue_size=10),
                            rospy.Subscriber("obs", MarkerArray, self.on_obstacles, queue_size=2)]
        self.timer = rospy.Timer(rospy.Duration(0.5), self.on_timer, reset=True)
        rospy.loginfo("Complex avoidance checker ready: %d gates, %d static and %d dynamic cylinders.",
                      len(self.gates), len(self.static), len(self.dynamic))

    def on_goal(self, message):
        p = message.pose.position
        if message.action == Marker.ADD and all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            with self.lock:
                self.goal = (p.x, p.y, p.z)
                self.goal_time = message.header.stamp.to_sec()

    def on_obstacles(self, message):
        with self.lock:
            if not self.finished:
                for marker in message.markers:
                    if marker.ns in self.dynamic:
                        self.dynamic[marker.ns].update(marker)

    def on_odom(self, message):
        with self.lock:
            if self.finished:
                return
            p = message.pose.pose.position
            t = message.header.stamp.to_sec()
            if not all(math.isfinite(v) for v in (t, p.x, p.y, p.z)) or message.header.frame_id.lstrip("/") != "world":
                self.finish(False, "invalid or non-world odometry")
                return
            if self.last_time is not None and t < self.last_time - 1e-6:
                self.finish(False, "simulation time reset; restart the checker")
                return
            if self.start_time is None:
                self.start_time = t
            self.last_time, self.last_position = t, [p.x, p.y, p.z]
            fresh = self.goal_time is not None and -0.2 <= t - self.goal_time <= 1.0
            if p.z <= 0:
                self.finish(False, "aircraft position reached the ground")
                return
            for name, (ox, oy, oz, r, h) in self.static:
                physical = cylinder_clearance(p.x, p.y, p.z, ox, oy, oz, r, h)
                inflated = cylinder_clearance(p.x, p.y, p.z, ox, oy, oz, 1.2 * r, h + 2.0)
                self.min_static_physical = min(self.min_static_physical, physical)
                self.min_static_inflated = min(self.min_static_inflated, inflated)
                self.static_samples += 1
                if physical < 0:
                    self.finish(False, "aircraft position intersected " + name)
                    return
            for moving in self.dynamic.values():
                clearance = moving.inspect(t, p.x, p.y, p.z, self.goal, fresh, self.encounter_distance)
                if clearance is None:
                    if t - self.start_time > 2.0:
                        self.dynamic_missing_samples += 1
                    continue
                if clearance[0] < 0:
                    self.finish(False, "aircraft position intersected " + moving.name)
                    return
            for gate in self.gates:
                if gate.observe(p.x, p.y, p.z, self.goal, fresh):
                    log = rospy.loginfo if gate.passed else rospy.logerr
                    log("Complex gate %s: %s; y %.2f..%.2f m, z %.2f..%.2f m; %s",
                        gate.name, "PASS" if gate.passed else "FAIL", gate.min_y, gate.max_y,
                        gate.min_height, gate.max_height, gate.reason)
                    if not gate.passed:
                        self.finish(False, "gate " + gate.name + ": " + gate.reason)
                        return
            if p.x >= self.final_x and abs(p.z - self.final_height) <= self.final_tolerance:
                reasons = []
                if not all(g.complete and g.passed for g in self.gates):
                    reasons.append("not all static gates were verified")
                if not fresh or abs(self.goal[2] - self.final_height) > 0.05:
                    reasons.append("final altitude goal unavailable or changed")
                if not all(d.max_motion > self.motion_min for d in self.dynamic.values()):
                    reasons.append("not all dynamic obstacles visibly moved")
                if not any(d.encounter_samples > 0 for d in self.dynamic.values()):
                    reasons.append("no close dynamic-obstacle encounter was observed")
                if not any(d.blocked_reference_samples > 0 for d in self.dynamic.values()):
                    reasons.append("no moving obstacle blocked the unchanged goal line")
                if self.dynamic_missing_samples:
                    reasons.append("dynamic markers were unavailable during part of the flight")
                self.finish(not reasons, "; ".join(reasons) if reasons else
                            "horizontal and vertical gates, moving-obstacle encounter and final altitude verified")

    def on_timer(self, _event):
        with self.lock:
            if self.finished:
                return
            t = rospy.Time.now().to_sec()
            if self.start_time is None:
                self.start_time = t
            elif t < self.start_time - 1e-6:
                self.finish(False, "simulation time reset; restart the checker")
            elif t - self.start_time > self.timeout:
                self.finish(False, "simulation-time timeout before all course evidence was collected")

    def finish(self, passed, reason):
        self.finished = True
        summary = {
            "status": "passed" if passed else "failed", "passed": passed, "reason": reason,
            "gates": [g.summary() for g in self.gates],
            "dynamic_obstacles": [d.summary() for d in self.dynamic.values()],
            "min_static_physical_clearance": self.min_static_physical if self.static_samples else 0.0,
            "min_static_inflated_clearance": self.min_static_inflated if self.static_samples else 0.0,
            "dynamic_missing_samples": self.dynamic_missing_samples,
            "last_position": self.last_position,
            "elapsed_simulation_seconds": max(0.0, (self.last_time or 0.0) - (self.start_time or 0.0)),
            "clearance_model": "odometry point against finite cylinders; dynamic centers time-aligned from markers",
        }
        rospy.set_param("~summary", summary)
        rospy.set_param("~passed", passed)
        if self.report_file:
            try:
                Path(self.report_file).write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
            except OSError as error:
                rospy.logerr("Cannot save complex validation report: %s", error)
        log = rospy.loginfo if passed else rospy.logerr
        log("Complex avoidance validation %s: %s", "PASS" if passed else "FAIL", reason)


if __name__ == "__main__":
    rospy.init_node("verify_complex_avoidance")
    try:
        ComplexAvoidanceChecker()
        rospy.spin()
    except (KeyError, ValueError, TypeError) as error:
        rospy.logfatal("Invalid complex validation configuration: %s", error)
        raise SystemExit(1)
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
