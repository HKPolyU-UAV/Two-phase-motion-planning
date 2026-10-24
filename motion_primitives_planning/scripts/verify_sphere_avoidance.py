#!/usr/bin/env python3
"""Observe sphere avoidance using the aircraft odometry point, without actuation.

A vertical bypass requires entering a sphere's XY footprint while staying clear
in 3-D and keeping the goal at the sphere's center altitude. Configured cylinder
radii can include existing planning margins; clearance is not a full-body test.
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
    return max(math.hypot(p[0] - c[0], p[1] - c[1]) - radius,
               abs(p[2] - c[2]) - height / 2)


def segment_distance(center, start, end):
    delta = tuple(b - a for a, b in zip(start, end))
    norm = sum(v * v for v in delta)
    if norm < 1e-10:
        return distance(center, start)
    fraction = max(0.0, min(1.0, sum((c - a) * d for c, a, d in zip(center, start, delta)) / norm))
    return distance(center, tuple(a + fraction * d for a, d in zip(start, delta)))


def measured(value):
    return value if math.isfinite(value) else 0.0


class SpherePassage:
    def __init__(self, config, obstacle, margin):
        self.name = config['name']
        self.center = xyz(obstacle[k] for k in ('x', 'y', 'z'))
        self.radius = float(obstacle['r'])
        if not math.isfinite(self.radius) or self.radius <= 0:
            raise ValueError("sphere radius must be positive and finite")
        self.inflated_radius = self.radius + max(.2 * self.radius, margin)
        self.goal, self.approach = xyz(config['goal']), xyz(config['approach'])
        dx, dy = self.goal[0] - self.approach[0], self.goal[1] - self.approach[1]
        length = math.hypot(dx, dy)
        if length < 1 or sum((g - c) * d for g, c, d in zip(self.goal[:2], self.center[:2], (dx, dy))) <= 0:
            raise ValueError("each goal must be beyond its sphere along the approach direction")
        self.direction = (dx / length, dy / length)
        self.armed = self.saw_before = self.complete = self.passed = False
        self.samples = self.encounter_samples = self.blocked_line_samples = self.vertical_samples = 0
        self.min_distance = self.min_xy = math.inf
        self.closest = []
        self.goal_verified = True
        self.previous = None
        self.continuous = True
        self.bypass = 'unobserved'
        self.reason = 'not yet passed'

    def observe(self, p, goal, fresh):
        if self.complete:
            return False
        matches = fresh and distance(goal, self.goal) < .1
        progress = sum((p[i] - self.center[i]) * self.direction[i] for i in (0, 1))
        if matches:
            self.armed = True
        if not self.armed:
            return False
        if progress < -self.radius:
            self.saw_before = True
        d = distance(p, self.center)
        horizontal = math.hypot(p[0] - self.center[0], p[1] - self.center[1])
        self.samples += 1
        if d < self.min_distance:
            self.min_distance, self.closest = d, list(p)
        self.min_xy = min(self.min_xy, horizontal)
        if d < self.radius + 15:
            self.encounter_samples += 1
            self.goal_verified = self.goal_verified and matches
            if self.previous is not None and distance(p, self.previous) > 2:
                self.continuous = False
        if matches and progress < 0 and segment_distance(self.center, p, goal) <= self.inflated_radius:
            self.blocked_line_samples += 1
        if (matches and abs(goal[2] - self.center[2]) < .1 and horizontal < self.radius
                and d > self.radius and abs(p[2] - self.center[2]) > 1):
            self.vertical_samples += 1
            self.bypass = 'above' if p[2] > self.center[2] else 'below'
        self.previous = p
        if progress <= self.radius + 2:
            return False
        self.complete = True
        failures = []
        if not self.saw_before or self.samples < 3 or not self.continuous:
            failures.append('insufficient continuous approach/pass observations')
        if not self.encounter_samples:
            failures.append('sphere was not approached closely')
        if not self.goal_verified:
            failures.append('expected goal changed or was unavailable during encounter')
        if self.min_distance <= self.radius:
            failures.append('aircraft point intersected sphere')
        self.passed = not failures
        if not self.vertical_samples and self.encounter_samples:
            self.bypass = 'side'
        self.reason = '; '.join(failures) if failures else 'sphere passed with the expected goal behind it'
        return True

    def summary(self):
        return dict(name=self.name, center=self.center, radius=self.radius, goal=self.goal,
                    complete=self.complete, passed=self.passed, bypass=self.bypass,
                    encounter_samples=self.encounter_samples, blocked_goal_segment_samples=self.blocked_line_samples,
                    vertical_bypass_samples=self.vertical_samples, goal_verified=self.goal_verified,
                    min_center_distance=measured(self.min_distance), min_xy_distance=measured(self.min_xy),
                    min_physical_clearance=measured(self.min_distance - self.radius),
                    min_inflated_clearance=measured(self.min_distance - self.inflated_radius),
                    closest_position=self.closest,
                    closest_height_offset=self.closest[2] - self.center[2] if self.closest else 0.0,
                    reason=self.reason)


class SphereAvoidanceChecker:
    def __init__(self):
        self.lock = threading.RLock()
        cfg = rospy.get_param('sphere_validation')
        spheres = rospy.get_param('sphere_obstacles')
        margin = float(rospy.get_param('motion_primitive/vertical_clearance', 1.0))
        obstacles = {'sphere_obs' + str(i): spheres['sphere' + str(i)] for i in range(1, spheres['num'] + 1)}
        self.cases = [SpherePassage(case, obstacles[case['name']], margin) for case in cfg['cases']]
        if not self.cases or len(self.cases) != len(obstacles) or {c.name for c in self.cases} != set(obstacles):
            raise ValueError('each configured sphere needs exactly one validation case')
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
        self.min_sphere_clearance = math.inf
        self.dynamic_missing_samples = 0
        self.report_file = rospy.get_param('~report_file', '')
        if rospy.has_param('~passed'):
            rospy.delete_param('~passed')
        rospy.set_param('~summary', dict(status='waiting', passed=False))
        self.subscribers = [rospy.Subscriber('goal', Marker, self.on_goal, queue_size=10),
                            rospy.Subscriber('obs', MarkerArray, self.on_obstacles, queue_size=2),
                            rospy.Subscriber('odom', Odometry, self.on_odom, queue_size=50)]
        self.timer = rospy.Timer(rospy.Duration(.5), self.on_timer, reset=True)
        rospy.loginfo('Sphere checker ready: %d suspended spheres; waiting for simulation playback.', len(self.cases))

    def on_goal(self, message):
        p = message.pose.position
        if message.action == Marker.ADD and all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            with self.lock:
                self.goal, self.goal_time = (p.x, p.y, p.z), message.header.stamp.to_sec()

    def on_obstacles(self, message):
        with self.lock:
            for m in message.markers:
                if m.ns not in self.dynamic or m.action != Marker.ADD or m.type != Marker.CYLINDER:
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
                clearance = distance(p, case.center) - case.radius
                self.min_sphere_clearance = min(self.min_sphere_clearance, clearance)
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
                    log('Sphere %s: %s, bypass=%s, minimum clearance %.2f m.', case.name,
                        'PASS' if case.passed else 'FAIL', case.bypass, case.min_distance - case.radius)
                    if not case.passed:
                        self.finish(False, case.name + ': ' + case.reason); return
            final = self.cases[-1].goal
            if (all(c.complete for c in self.cases) and fresh and distance(self.goal, final) < .1
                    and math.hypot(p[0] - final[0], p[1] - final[1]) <= self.final_tolerance
                    and abs(p[2] - final[2]) <= self.height_tolerance):
                reasons = []
                if not any(c.vertical_samples > 0 for c in self.cases):
                    reasons.append('all observed sphere passages were lateral; vertical bypass not demonstrated')
                if not any(c.blocked_line_samples > 0 for c in self.cases):
                    reasons.append('no active goal segment was observed blocked by a sphere')
                if not all(d['motion'] > 2 and d['checks'] > 0 for d in self.dynamic.values()):
                    reasons.append('not all original dynamic cylinders were observed moving')
                if self.dynamic_missing_samples:
                    reasons.append('dynamic obstacle observations were missing during part of the flight')
                self.finish(not reasons, '; '.join(reasons) if reasons else
                            'sphere passages, true vertical bypass, moving cylinders and final target verified')

    def on_timer(self, _event):
        with self.lock:
            if self.finished:
                return
            now = rospy.Time.now().to_sec()
            if self.start_time is None:
                self.start_time = now
            elif now - self.start_time > self.timeout:
                self.finish(False, 'simulation-time timeout before all sphere passages and final target')

    def finish(self, passed, reason):
        self.finished = True
        summary = dict(status='passed' if passed else 'failed', passed=passed, reason=reason,
                       spheres=[c.summary() for c in self.cases], last_position=self.position,
                       elapsed_simulation_seconds=max(0.0, (self.last_time or 0) - (self.start_time or 0)),
                       min_sphere_physical_clearance=measured(self.min_sphere_clearance),
                       min_configured_cylinder_clearance=measured(self.min_cylinder_clearance),
                       dynamic_missing_samples=self.dynamic_missing_samples,
                       dynamic_obstacles=[dict(name=n, max_observed_motion=d['motion'], checked_samples=d['checks'],
                                               min_physical_clearance=measured(d['minimum'])) for n, d in self.dynamic.items()],
                       clearance_model='odometry point; spheres use Euclidean distance, cylinders use configured radii')
        rospy.set_param('~passed', passed); rospy.set_param('~summary', summary)
        if self.report_file:
            try:
                Path(self.report_file).write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
            except OSError as error:
                rospy.logerr('Cannot write sphere validation report: %s', error)
        (rospy.loginfo if passed else rospy.logerr)('Sphere validation %s: %s', 'PASS' if passed else 'FAIL', reason)


if __name__ == '__main__':
    rospy.init_node('verify_sphere_avoidance')
    try:
        SphereAvoidanceChecker()
        rospy.spin()
    except (ValueError, KeyError) as error:
        rospy.logfatal('Invalid sphere validation configuration: %s', error)
        raise SystemExit(1)
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
