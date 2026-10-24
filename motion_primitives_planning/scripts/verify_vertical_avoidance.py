#!/usr/bin/env python3
"""Observe the three-dimensional validation course without commanding the plane.

The checker distinguishes goal-height tracking from obstacle avoidance: the
active goal remains inside each wall's blocked height band while the aircraft
passes above/below that wall. All distances and heights are world coordinates.
Results are exposed as private ROS parameters ``passed`` and ``summary``.
"""

import json
import math
import threading
from pathlib import Path

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class GateObservation:
    """Evidence for the first forward passage through a wall's inflated slab."""

    def __init__(self, config, radius, half_width, altitude_tolerance):
        self.name = str(config["name"])
        self.x = float(config["x"])
        self.direction = config["direction"]
        if self.direction not in ("above", "below"):
            raise ValueError("gate direction must be 'above' or 'below'")
        self.threshold = float(config["threshold"])
        self.goal_height = float(config["goal_height"])
        self.start_height = float(config["start_height"])
        if not all(math.isfinite(v) for v in
                   (self.x, self.threshold, self.goal_height, self.start_height)):
            raise ValueError("gate coordinates/heights must be finite")
        self.radius = radius
        self.half_width = half_width
        self.altitude_tolerance = altitude_tolerance
        self.saw_before = False
        self.approach_verified = False
        self.samples = 0
        self.min_height = math.inf
        self.max_height = -math.inf
        self.max_abs_y = 0.0
        self.min_clearance = math.inf
        self.goal_verified = True
        self.continuous = True
        self.previous_x = None
        self.complete = False
        self.passed = False
        self.reason = "not yet crossed"

    def observe(self, x, y, z, goal_height, goal_fresh):
        if self.complete:
            return False
        front, rear = self.x - self.radius, self.x + self.radius
        goal_matches = goal_fresh and abs(goal_height - self.goal_height) <= 0.05
        if self.x - 65.0 <= x <= self.x - 30.0:
            if abs(z - self.start_height) <= self.altitude_tolerance and goal_matches:
                self.approach_verified = True
        if x < front:
            self.saw_before = True
        if front <= x <= rear:
            self.samples += 1
            self.min_height = min(self.min_height, z)
            self.max_height = max(self.max_height, z)
            self.max_abs_y = max(self.max_abs_y, abs(y))
            clearance = z - self.threshold if self.direction == "above" else self.threshold - z
            self.min_clearance = min(self.min_clearance, clearance)
            self.goal_verified = self.goal_verified and goal_matches
        # Missing a substantial section of the wall cannot count as clearance.
        if self.previous_x is not None and self.previous_x <= rear and x >= front:
            if abs(x - self.previous_x) > 2.0:
                self.continuous = False
        self.previous_x = x
        if x <= rear:
            return False
        self.complete = True
        failures = []
        if not self.saw_before or self.samples < 3 or not self.continuous:
            failures.append("insufficient continuous passage observations")
        if not self.approach_verified:
            failures.append("approach at the blocked goal height was not observed")
        if self.max_abs_y >= self.half_width:
            failures.append("lateral bypass outside the validation corridor")
        if not self.goal_verified:
            failures.append("active goal height changed or was unavailable during passage")
        if self.min_clearance <= 0:
            failures.append("vertical clearance threshold was violated")
        self.passed = not failures
        self.reason = "; ".join(failures) if failures else "vertical passage verified at an obstructed goal height"
        return True

    def summary(self):
        return {
            "name": self.name, "x": self.x, "direction": self.direction,
            "threshold": self.threshold, "goal_height": self.goal_height,
            "complete": self.complete, "passed": self.passed,
            "samples": self.samples, "approach_verified": self.approach_verified,
            "goal_verified": self.goal_verified and self.samples > 0,
            "min_height": self.min_height if self.samples else 0.0,
            "max_height": self.max_height if self.samples else 0.0,
            "min_clearance": self.min_clearance if self.samples else 0.0,
            "max_abs_y": self.max_abs_y, "reason": self.reason,
        }


class VerticalAvoidanceChecker:
    def __init__(self):
        self._lock = threading.RLock()
        prefix = "vertical_validation/"
        radius = float(rospy.get_param(prefix + "radius", 7.2))
        half_width = float(rospy.get_param(prefix + "half_width", 80.0))
        altitude_tolerance = float(rospy.get_param(prefix + "altitude_tolerance", 3.0))
        self.final_x = float(rospy.get_param(prefix + "final_x", 400.0))
        self.final_height = float(rospy.get_param(prefix + "final_height", 8.0))
        self.final_tolerance = float(rospy.get_param(prefix + "final_tolerance", 3.0))
        self.deadline = float(rospy.get_param(prefix + "timeout", 60.0))
        if not all(math.isfinite(v) and v > 0 for v in
                   (radius, half_width, altitude_tolerance, self.final_tolerance, self.deadline)):
            raise ValueError("validation dimensions/tolerances/timeout must be positive and finite")
        if not math.isfinite(self.final_x) or not math.isfinite(self.final_height):
            raise ValueError("final position and height must be finite")
        self.gates = [GateObservation(c, radius, half_width, altitude_tolerance)
                      for c in rospy.get_param(prefix + "gates")]
        if not self.gates or len({g.name for g in self.gates}) != len(self.gates):
            raise ValueError("validation gates must be non-empty and have unique names")
        self.report_file = rospy.get_param("~report_file", "")
        self.goal_height = 0.0
        self.goal_time = None
        self.start_time = None
        self.last_time = None
        self.last_position = []
        self.finished = False
        if rospy.has_param("~passed"):
            rospy.delete_param("~passed")
        rospy.set_param("~summary", {"status": "waiting", "passed": False})
        self._goal_sub = rospy.Subscriber("goal", Marker, self._on_goal, queue_size=10)
        self._odom_sub = rospy.Subscriber("odom", Odometry, self._on_odom, queue_size=50)
        self._timer = rospy.Timer(rospy.Duration(0.5), self._on_timer, reset=True)
        rospy.loginfo("Vertical avoidance checker ready: %s. Waiting for simulation playback.",
                      ", ".join(g.name for g in self.gates))

    def _on_goal(self, message):
        if message.action != Marker.ADD or not math.isfinite(message.pose.position.z):
            return
        with self._lock:
            self.goal_height = message.pose.position.z
            self.goal_time = message.header.stamp.to_sec()

    def _on_odom(self, message):
        with self._lock:
            if self.finished:
                return
            p = message.pose.pose.position
            timestamp = message.header.stamp.to_sec()
            if not all(math.isfinite(v) for v in (timestamp, p.x, p.y, p.z)):
                self._finish(False, "non-finite odometry")
                return
            if message.header.frame_id.lstrip("/") != "world":
                self._finish(False, "odometry is not expressed in the world frame")
                return
            if self.last_time is not None and timestamp < self.last_time - 1e-6:
                self._finish(False, "simulation time reset during validation; restart the checker")
                return
            if self.start_time is None:
                self.start_time = timestamp
            self.last_time = timestamp
            self.last_position = [p.x, p.y, p.z]
            fresh = self.goal_time is not None and -0.1 <= timestamp - self.goal_time <= 1.0
            for gate in self.gates:
                if gate.observe(p.x, p.y, p.z, self.goal_height, fresh):
                    log = rospy.loginfo if gate.passed else rospy.logerr
                    log("Vertical gate %s: %s; height %.2f..%.2f m, minimum clearance %.2f m, %s",
                        gate.name, "PASS" if gate.passed else "FAIL",
                        gate.min_height if gate.samples else 0.0,
                        gate.max_height if gate.samples else 0.0,
                        gate.min_clearance if gate.samples else 0.0, gate.reason)
                    if not gate.passed:
                        self._finish(False, "gate {} failed: {}".format(gate.name, gate.reason))
                        return
            if (all(g.complete and g.passed for g in self.gates) and p.x >= self.final_x
                    and abs(p.z - self.final_height) <= self.final_tolerance
                    and fresh and abs(self.goal_height - self.final_height) <= 0.05):
                self._finish(True, "both vertical passages and the final lower-altitude target verified")

    def _on_timer(self, _event):
        with self._lock:
            if self.finished:
                return
            now = rospy.Time.now().to_sec()
            if self.start_time is None:
                self.start_time = now
            elif now < self.start_time - 1e-6:
                self._finish(False, "simulation time reset during validation; restart the checker")
            elif now - self.start_time > self.deadline:
                pending = [g.name for g in self.gates if not g.complete]
                detail = "missing gate passages: " + ", ".join(pending) if pending else "final lower-altitude target not reached"
                self._finish(False, "simulation-time timeout; " + detail)

    def _finish(self, passed, reason):
        self.finished = True
        summary = {
            "status": "passed" if passed else "failed", "passed": passed,
            "reason": reason, "gates": [g.summary() for g in self.gates],
            "last_position": self.last_position,
            "elapsed_simulation_seconds": max(0.0, (self.last_time or 0.0) - (self.start_time or 0.0)),
        }
        rospy.set_param("~passed", passed)
        rospy.set_param("~summary", summary)
        if self.report_file:
            try:
                Path(self.report_file).write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
            except OSError as error:
                rospy.logerr("Cannot save vertical validation report: %s", error)
        log = rospy.loginfo if passed else rospy.logerr
        log("Vertical avoidance validation %s: %s", "PASS" if passed else "FAIL", reason)


if __name__ == "__main__":
    rospy.init_node("verify_vertical_avoidance")
    try:
        VerticalAvoidanceChecker()
        rospy.spin()
    except (ValueError, KeyError) as error:
        rospy.logfatal("Invalid vertical validation configuration: %s", error)
        raise SystemExit(1)
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
