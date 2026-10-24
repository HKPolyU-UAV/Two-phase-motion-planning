#!/usr/bin/env python3
"""Observe vertical passage over/under long horizontal cylinders without actuation.

The centre-plane crossing must clear the rod radius and safety margin, away from
both end caps. Configured cylinder radii can contain existing planning margins;
all clearance checks use the odometry point rather than a full-aircraft volume.
"""
import json
import math
import threading
from pathlib import Path

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


def xyz(values):
    result = tuple(float(v) for v in values)
    if len(result) != 3 or not all(math.isfinite(v) for v in result):
        raise ValueError("expected three finite coordinates")
    return result


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def cylinder_clearance(p, c, radius, height):
    radial = math.hypot(p[0] - c[0], p[1] - c[1]) - radius
    cap = abs(p[2] - c[2]) - height / 2
    return math.hypot(max(radial, 0.0), max(cap, 0.0)) + min(max(radial, cap), 0.0)


def measured(value):
    return value if math.isfinite(value) else 0.0


class RodPassage:
    """Observe passage through the centre of a finite, horizontal cylinder."""
    def __init__(self, config, obstacle, margin, end_clearance=10.0):
        self.name = config['name']
        self.center = xyz(obstacle[k] for k in ('x', 'y', 'z'))
        self.radius, self.length = float(obstacle['r']), float(obstacle['length'])
        angle = math.radians(float(obstacle['axis_yaw']))
        if not all(math.isfinite(v) for v in (self.radius, self.length, angle, margin)) or min(self.radius, self.length) <= 0:
            raise ValueError('rod dimensions and orientation must be finite and positive')
        self.padding = max(.2 * self.radius, margin)
        self.inflated_radius = self.radius + self.padding
        self.axis = (math.cos(angle), math.sin(angle))
        self.normal = (-self.axis[1], self.axis[0])
        self.goal, self.approach = xyz(config['goal']), xyz(config['approach'])
        forward = sum((self.goal[i] - self.approach[i]) * self.normal[i] for i in (0, 1))
        if abs(forward) < 1:
            raise ValueError('approach must cross the rod axis')
        if forward < 0:
            self.normal = tuple(-v for v in self.normal)
        if self.coordinates(self.goal)[0] <= self.inflated_radius:
            raise ValueError('goal must lie beyond the rod')
        if abs(self.goal[2] - self.center[2]) >= self.radius:
            raise ValueError('goal altitude must remain inside the rod blocked height band')
        self.expected = config.get('direction', 'either')
        if self.expected not in ('above', 'below', 'either'):
            raise ValueError('direction must be above, below or either')
        end_clearance = float(end_clearance)
        if not math.isfinite(end_clearance) or end_clearance < 10.0:
            raise ValueError('end_clearance must be at least 10 m')
        self.end_limit = self.length / 2 - end_clearance
        if self.end_limit <= 0:
            raise ValueError('validation rods need more than 20 m length')
        self.armed = self.saw_before = self.complete = self.passed = False
        self.samples = 0
        self.min_physical = self.min_inflated = self.min_height = math.inf
        self.max_height = -math.inf
        self.max_abs_axial = 0.0
        self.closest = []
        self.goal_verified = self.continuous = True
        self.previous = self.crossing = None
        self.bypass = 'unobserved'
        self.reason = 'not yet passed'

    def coordinates(self, p):
        dx, dy = p[0] - self.center[0], p[1] - self.center[1]
        return (dx * self.normal[0] + dy * self.normal[1],
                dx * self.axis[0] + dy * self.axis[1], p[2] - self.center[2])

    def clearance(self, p, padding=0.0):
        normal, axial, height = self.coordinates(p)
        radial_gap = math.hypot(normal, height) - self.radius - padding
        cap_gap = abs(axial) - self.length / 2 - padding
        # Exact signed distance to a finite cylinder, including its flat caps.
        return math.hypot(max(radial_gap, 0.0), max(cap_gap, 0.0)) + min(max(radial_gap, cap_gap), 0.0)

    def inspect(self, p):
        physical, inflated = self.clearance(p), self.clearance(p, self.padding)
        if physical < self.min_physical:
            self.min_physical, self.closest = physical, list(p)
        self.min_inflated = min(self.min_inflated, inflated)
        return physical

    def observe(self, p, goal, fresh):
        if self.complete:
            return False
        matches = fresh and distance(goal, self.goal) < .1
        progress, axial, height = self.coordinates(p)
        if matches:
            self.armed = True
        if not self.armed:
            return False
        if progress < -self.inflated_radius:
            self.saw_before = True
        if abs(progress) <= self.inflated_radius:
            self.samples += 1
            self.min_height, self.max_height = min(self.min_height, p[2]), max(self.max_height, p[2])
            self.max_abs_axial = max(self.max_abs_axial, abs(axial))
            self.goal_verified = self.goal_verified and matches
        if self.previous is not None:
            old_progress, old_axial, old_height = self.coordinates(self.previous)
            if old_progress <= self.inflated_radius and progress >= -self.inflated_radius and distance(p, self.previous) > 2:
                self.continuous = False
            if self.crossing is None and old_progress < 0 <= progress:
                fraction = -old_progress / (progress - old_progress)
                crossing_axial = old_axial + fraction * (axial - old_axial)
                crossing_height = old_height + fraction * (height - old_height)
                self.crossing = (crossing_axial, crossing_height)
                self.bypass = 'above' if crossing_height > 0 else 'below'
        self.previous = p
        if progress <= self.inflated_radius:
            return False
        self.complete = True
        failures = []
        if not self.saw_before or self.samples < 3 or not self.continuous or self.crossing is None:
            failures.append('insufficient continuous approach and centre crossing observations')
        if not self.goal_verified:
            failures.append('expected goal changed or was unavailable during passage')
        if self.max_abs_axial > self.end_limit or (self.crossing is not None and abs(self.crossing[0]) > self.end_limit):
            failures.append('rod end bypass instead of vertical passage')
        if self.crossing is not None:
            if abs(self.crossing[1]) <= self.inflated_radius:
                failures.append('centre crossing does not clear the vertical safety envelope')
            if self.expected != 'either' and self.bypass != self.expected:
                failures.append('expected ' + self.expected + ' passage, observed ' + self.bypass)
        if self.min_physical <= 0:
            failures.append('aircraft point intersected the rod')
        self.passed = not failures
        self.reason = '; '.join(failures) if failures else 'vertical passage verified with an unchanged blocked-height goal'
        return True

    def summary(self):
        return dict(name=self.name, center=self.center, radius=self.radius, length=self.length,
                    goal=self.goal, expected_direction=self.expected, complete=self.complete, passed=self.passed,
                    bypass=self.bypass, samples=self.samples, goal_verified=self.goal_verified,
                    min_physical_clearance=measured(self.min_physical), min_inflated_clearance=measured(self.min_inflated),
                    closest_position=self.closest, min_passage_height=measured(self.min_height),
                    max_passage_height=measured(self.max_height), max_abs_axial_position=self.max_abs_axial,
                    center_crossing_height=self.center[2] + self.crossing[1] if self.crossing else 0.0,
                    center_crossing_axial_position=self.crossing[0] if self.crossing else 0.0,
                    center_crossing_vertical_margin=abs(self.crossing[1]) - self.inflated_radius if self.crossing else 0.0,
                    reason=self.reason)


class HorizontalAvoidanceChecker:
    def __init__(self):
        self.lock = threading.RLock()
        cfg = rospy.get_param('horizontal_validation')
        rods = rospy.get_param('horizontal_obstacles')
        margin = float(rospy.get_param('motion_primitive/vertical_clearance', 1.0))
        obstacles = {'horizontal_obs' + str(i): rods['cylinder' + str(i)] for i in range(1, rods['num'] + 1)}
        self.cases = [RodPassage(case, obstacles[case['name']], margin, cfg.get('end_clearance', 10.0))
                      for case in cfg['cases']]
        if not self.cases or len(self.cases) != len(obstacles) or {c.name for c in self.cases} != set(obstacles):
            raise ValueError('each configured horizontal rod needs exactly one validation case')
        self.timeout = float(cfg.get('timeout', 120))
        self.final_tolerance = float(cfg.get('final_tolerance', 55))
        self.height_tolerance = float(cfg.get('height_tolerance', 3))
        if not all(math.isfinite(v) and v > 0 for v in (self.timeout, self.final_tolerance, self.height_tolerance)):
            raise ValueError('validation tolerances and timeout must be positive')
        static = rospy.get_param('static_obstacles')
        self.cylinders = []
        for i in range(1, static['num'] + 1):
            o = static['obs' + str(i)]; h = float(o.get('height', static.get('height', 36.2743)))
            self.cylinders.append((xyz((o['x'], o['y'], o.get('z', h / 2))), float(o['r']), h))
        dyn = rospy.get_param('dynamic_obstacles')
        self.dynamic = {'dyn_obs' + str(i): dict(config=dyn['dyn_obs' + str(i)], marker=None,
                                               origin=None, motion=0.0, checks=0, minimum=math.inf)
                        for i in range(1, dyn['num'] + 1)}
        self.goal = (0.0, 0.0, 0.0)
        self.goal_time = self.start_time = self.last_time = None
        self.position = []
        self.finished = False
        self.min_cylinder_clearance = math.inf
        self.min_rod_clearance = math.inf
        self.dynamic_missing_samples = 0
        self.report_file = rospy.get_param('~report_file', '')
        if rospy.has_param('~passed'):
            rospy.delete_param('~passed')
        rospy.set_param('~summary', dict(status='waiting', passed=False))
        self.subscribers = [rospy.Subscriber('goal', Marker, self.on_goal, queue_size=10),
                            rospy.Subscriber('obs', MarkerArray, self.on_obstacles, queue_size=2),
                            rospy.Subscriber('odom', Odometry, self.on_odom, queue_size=50)]
        self.timer = rospy.Timer(rospy.Duration(.5), self.on_timer, reset=True)
        rospy.loginfo('Horizontal-cylinder checker ready: %d rods; waiting for simulation playback.', len(self.cases))

    def on_goal(self, message):
        p = message.pose.position
        if (message.action == Marker.ADD and message.header.frame_id.lstrip('/') == 'world'
                and all(math.isfinite(v) for v in (p.x, p.y, p.z))):
            with self.lock:
                self.goal, self.goal_time = (p.x, p.y, p.z), message.header.stamp.to_sec()

    def on_obstacles(self, message):
        with self.lock:
            for m in message.markers:
                if (m.ns not in self.dynamic or m.action != Marker.ADD or m.type != Marker.CYLINDER
                        or m.header.frame_id.lstrip('/') != 'world'):
                    continue
                p = m.pose.position
                sample = (m.header.stamp.to_sec(), p.x, p.y, p.z, m.scale.x / 2, m.scale.z)
                if not all(math.isfinite(v) for v in sample) or min(sample[4:]) <= 0:
                    continue
                d = self.dynamic[m.ns]
                if d['origin'] is None:
                    d['origin'] = (p.x, p.y)
                d['motion'] = max(d['motion'], math.hypot(p.x - d['origin'][0], p.y - d['origin'][1]))
                d['marker'] = sample

    def dynamic_clearance(self, d, t, p):
        m = d['marker']
        if m is None or abs(t - m[0]) > 1:
            return None
        dt = t - m[0]; _, x, y, z, r, h = m; c = d['config']
        if c.get('type', 'linear') == 'circular':
            cx, cy = c['center']['x'], c['center']['y']
            angle = math.atan2(y - cy, x - cx) + c['angular_velocity'] * dt
            x, y = cx + c['radius'] * math.cos(angle), cy + c['radius'] * math.sin(angle)
        else:
            angle = math.radians(c.get('direction', 0))
            x += c.get('velocity', 0) * math.cos(angle) * dt
            y += c.get('velocity', 0) * math.sin(angle) * dt
        clearance = cylinder_clearance(p, (x, y, z), r, h)
        d['checks'] += 1; d['minimum'] = min(d['minimum'], clearance)
        return clearance

    def on_odom(self, message):
        with self.lock:
            if self.finished:
                return
            q = message.pose.pose.position; p = (q.x, q.y, q.z); t = message.header.stamp.to_sec()
            if (not all(math.isfinite(v) for v in (*p, t)) or message.header.frame_id.lstrip('/') != 'world'
                    or p[2] <= 0):
                self.finish(False, 'invalid odometry or ground intersection'); return
            if self.last_time is not None and t < self.last_time - 1e-6:
                self.finish(False, 'simulation time reset; restart the checker'); return
            if self.start_time is None:
                self.start_time = t
            self.last_time, self.position = t, list(p)
            fresh = self.goal_time is not None and -.2 <= t - self.goal_time <= 1
            for case in self.cases:
                clearance = case.inspect(p)
                self.min_rod_clearance = min(self.min_rod_clearance, clearance)
                if clearance <= 0:
                    self.finish(False, 'aircraft point intersected ' + case.name); return
            for i, (center, r, h) in enumerate(self.cylinders, 1):
                clearance = cylinder_clearance(p, center, r, h)
                self.min_cylinder_clearance = min(self.min_cylinder_clearance, clearance)
                if clearance <= 0:
                    self.finish(False, 'aircraft point intersected configured static_obs' + str(i)); return
            for name, d in self.dynamic.items():
                clearance = self.dynamic_clearance(d, t, p)
                if clearance is None and t - self.start_time > 2:
                    self.dynamic_missing_samples += 1
                if clearance is not None and clearance <= 0:
                    self.finish(False, 'aircraft point intersected ' + name); return
            for case in self.cases:
                if case.observe(p, self.goal, fresh):
                    log = rospy.loginfo if case.passed else rospy.logerr
                    log('Horizontal rod %s: %s, bypass=%s, minimum clearance %.2f m.', case.name,
                        'PASS' if case.passed else 'FAIL', case.bypass, case.min_physical)
                    if not case.passed:
                        self.finish(False, case.name + ': ' + case.reason); return
            final = self.cases[-1].goal
            if (all(c.complete for c in self.cases) and fresh and distance(self.goal, final) < .1
                    and math.hypot(p[0] - final[0], p[1] - final[1]) <= self.final_tolerance
                    and abs(p[2] - final[2]) <= self.height_tolerance):
                reasons = []
                if not all(c.passed for c in self.cases):
                    reasons.append('not every rod was crossed vertically')
                if not {'above', 'below'}.issubset({c.bypass for c in self.cases}):
                    reasons.append('both an upward and a downward bypass must be demonstrated')
                if not all(d['motion'] > 2 and d['checks'] > 0 for d in self.dynamic.values()):
                    reasons.append('not all original dynamic cylinders were observed moving')
                if self.dynamic_missing_samples:
                    reasons.append('dynamic obstacle observations were missing during part of the flight')
                self.finish(not reasons, '; '.join(reasons) if reasons else
                            'all rods crossed vertically, upward and downward avoidance and moving cylinders verified')

    def on_timer(self, _event):
        with self.lock:
            if self.finished:
                return
            now = rospy.Time.now().to_sec()
            if self.start_time is None:
                self.start_time = now
            elif now - self.start_time > self.timeout:
                self.finish(False, 'simulation-time timeout before all vertical rod passages and final target')

    def finish(self, passed, reason):
        self.finished = True
        summary = dict(status='passed' if passed else 'failed', passed=passed, reason=reason,
                       rods=[c.summary() for c in self.cases], last_position=self.position,
                       elapsed_simulation_seconds=max(0.0, (self.last_time or 0) - (self.start_time or 0)),
                       min_rod_physical_clearance=measured(self.min_rod_clearance),
                       min_configured_cylinder_clearance=measured(self.min_cylinder_clearance),
                       dynamic_missing_samples=self.dynamic_missing_samples,
                       dynamic_obstacles=[dict(name=n, max_observed_motion=d['motion'], checked_samples=d['checks'],
                                               min_physical_clearance=measured(d['minimum'])) for n, d in self.dynamic.items()],
                       clearance_model='odometry point; finite horizontal cylinders with flat caps, original cylinders use configured radii')
        rospy.set_param('~passed', passed); rospy.set_param('~summary', summary)
        if self.report_file:
            try:
                Path(self.report_file).write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
            except OSError as error:
                rospy.logerr('Cannot write horizontal-cylinder validation report: %s', error)
        (rospy.loginfo if passed else rospy.logerr)('Horizontal-cylinder validation %s: %s', 'PASS' if passed else 'FAIL', reason)


if __name__ == '__main__':
    rospy.init_node('verify_horizontal_avoidance')
    try:
        HorizontalAvoidanceChecker()
        rospy.spin()
    except (ValueError, KeyError) as error:
        rospy.logfatal('Invalid horizontal-cylinder validation configuration: %s', error)
        raise SystemExit(1)
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
