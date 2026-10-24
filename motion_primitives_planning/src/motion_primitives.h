#ifndef MOTION_PRIMITIVES_H
#define MOTION_PRIMITIVES_H

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#define Rad2Deg 0.017453292519943295f

class MotionPrimitives {
public:
    struct waypoint {
        float d = 0, theata = 0, h = 0, V = 0;
    };
    typedef waypoint wp_t;

    struct primitive {
        float w_roll_c = 0, attack_c = 0, T_c = 0, t_way = 0;
        wp_t wp;
    };
    typedef primitive pr_t;

    // The Gazebo plugin uses custom odometry fields: V, gamma, sigma, T,
    // yaw rate. Its command fields are roll, attack angle and thrust.
    struct state {
        float V = 0, gamma = 0, sigma = 0, d_sigma = 0, T = 0, h = 0;
        float pos[2] = {0, 0};
        float q[4] = {1, 0, 0, 0};
        float roll = 0, attack = 0;
    };
    typedef state s_t;

    struct target {
        float pos[2] = {0, 0};
        float h = 5;
    };
    typedef target target_t;

    struct obstacle {
        float pos[2] = {0, 0};
        float r = 10, z = 0, height = 0;
    };
    typedef obstacle obs_t;

    struct sphere_obstacle {
        float x = 0, y = 0, z = 0, r = 5;
    };

    struct horizontal_obstacle {
        float x = 0, y = 0, z = 0, r = 4, length = 200, yaw = 0;
        float axis_x = 1, axis_y = 0;
    };

    struct dynamic_obstacle {
        float pos[2] = {0, 0}, current_pos[2] = {0, 0}, center[2] = {0, 0};
        float r = 5, z = 5, height = 10;
        std::string type;
        float velocity = 0, direction = 0, radius = 0, angular_velocity = 0, angle = 0;
        ros::Time last_update_time;
    };
    typedef dynamic_obstacle dyn_obs_t;

    struct rollout_t {
        std::vector<s_t> states;
        bool valid = true;
        bool collision = false;
    };

    explicit MotionPrimitives(ros::NodeHandle node) : nh(node) {
        nh.param<float>("model/CL/k1", CL_k1, 0.03f);
        nh.param<float>("model/CL/k2", CL_k2, 0.028f);
        nh.param<float>("model/CD/k1", CD_k1, 0.001f);
        nh.param<float>("model/CD/k2", CD_k2, 0.002f);
        nh.param<float>("motion_primitive/horizon_time", horizon_time, 3.0f);
        nh.param<float>("motion_primitive/V", mp_V, 15.5f);
        nh.param<int>("motion_primitive/num", mp_num, 29);
        nh.param<int>("motion_primitive/vertical_num", vertical_num, 7);
        load_angle("theata", mp_theata, 45);
        load_angle("max_roll_angle", mp_max_roll, 35);
        load_angle("max_attack_angle", mp_max_attack, 25);
        load_angle("min_attack_angle", mp_min_attack, -20);
        load_angle("max_climb_angle", max_climb_angle, 20);
        load_angle("max_descent_angle", max_descent_angle, 20);
        nh.param<float>("motion_primitive/compensate_tau", tau_a, 0.8f);
        nh.param<float>("motion_primitive/K_T", mp_k_T, 2.2f);
        nh.param<float>("motion_primitive/min_altitude", min_altitude, 1.0f);
        nh.param<float>("motion_primitive/max_altitude", max_altitude, 80.0f);
        nh.param<float>("motion_primitive/vertical_clearance", vertical_clearance, 1.0f);
        nh.param<float>("motion_primitive/integration_dt", integration_dt, 0.02f);
        nh.param<float>("motion_primitive/min_speed", min_speed, 3.0f);
        nh.param<float>("motion_primitive/max_speed", max_speed, 45.0f);
        nh.param<float>("motion_primitive/max_thrust", max_thrust, 20.0f);
        nh.param<float>("motion_primitive/altitude_weight", altitude_weight, 0.15f);
        nh.param<float>("motion_primitive/distance_weight", distance_weight, 0.02f);
        nh.param<float>("motion_primitive/heading_weight", heading_weight, 3.0f);
        nh.param<float>("motion_primitive/waypoint_altitude_tolerance", altitude_tolerance, 3.0f);
        mp_num = std::max(1, mp_num);
        vertical_num = std::max(1, vertical_num);
        if (vertical_num > 1 && vertical_num % 2 == 0) ++vertical_num;
        horizon_time = constraint(horizon_time, 0.1f, 10.0f);
        integration_dt = constraint(integration_dt, 0.005f, 0.05f);
        min_speed = std::max(1.0f, min_speed);
        max_speed = std::max(min_speed + 1.0f, max_speed);
        max_altitude = std::max(min_altitude + 1.0f, max_altitude);
        max_thrust = std::max(0.0f, max_thrust);
        vertical_clearance = std::max(0.0f, vertical_clearance);
        max_climb_angle = constraint(max_climb_angle, 0.0f, 45.0f * Rad2Deg);
        max_descent_angle = constraint(max_descent_angle, 0.0f, 45.0f * Rad2Deg);
        mp_d = horizon_time * mp_V;

        nh.param<int>("waypoints/num", wp_n, 1);
        wp_n = std::max(1, wp_n);
        wp_vec.resize(wp_n);
        for (int i = 0; i < wp_n; ++i) {
            const std::string prefix = "waypoints/wp" + std::to_string(i + 1);
            nh.param<float>(prefix + "/x", wp_vec[i].pos[0], 300.0f);
            nh.param<float>(prefix + "/y", wp_vec[i].pos[1], 0.0f);
            nh.param<float>(prefix + "/h", wp_vec[i].h, 5.0f);
        }
        global_target = wp_vec.front();

        int count;
        float default_height;
        nh.param<int>("static_obstacles/num", count, 0);
        nh.param<float>("static_obstacles/height", default_height, 36.2743f);
        static_obs_vec.resize(std::max(0, count));
        for (int i = 0; i < count; ++i) {
            const std::string prefix = "static_obstacles/obs" + std::to_string(i + 1);
            obs_t& obs = static_obs_vec[i];
            nh.param<float>(prefix + "/x", obs.pos[0], 110.0f);
            nh.param<float>(prefix + "/y", obs.pos[1], 10.0f);
            nh.param<float>(prefix + "/r", obs.r, 10.0f);
            nh.param<float>(prefix + "/height", obs.height, default_height);
            nh.param<float>(prefix + "/z", obs.z, obs.height / 2.0f);
        }
        nh.param<int>("sphere_obstacles/num", count, 0);
        sphere_obs_vec.resize(std::max(0, count));
        for (int i = 0; i < count; ++i) {
            const std::string prefix = "sphere_obstacles/sphere" + std::to_string(i + 1);
            auto& obs = sphere_obs_vec[i];
            nh.param<float>(prefix + "/x", obs.x, 0.0f);
            nh.param<float>(prefix + "/y", obs.y, 0.0f);
            nh.param<float>(prefix + "/z", obs.z, 10.0f);
            nh.param<float>(prefix + "/r", obs.r, 5.0f);
        }
        nh.param<int>("horizontal_obstacles/num", count, 0);
        horizontal_obs_vec.resize(std::max(0, count));
        for (int i = 0; i < count; ++i) {
            const std::string prefix = "horizontal_obstacles/cylinder" + std::to_string(i + 1);
            auto& obs = horizontal_obs_vec[i];
            nh.param<float>(prefix + "/x", obs.x, 0.0f);
            nh.param<float>(prefix + "/y", obs.y, 0.0f);
            nh.param<float>(prefix + "/z", obs.z, 10.0f);
            nh.param<float>(prefix + "/r", obs.r, 4.0f);
            nh.param<float>(prefix + "/length", obs.length, 200.0f);
            nh.param<float>(prefix + "/axis_yaw", obs.yaw, 0.0f);
            obs.yaw *= Rad2Deg;
            obs.axis_x = std::cos(obs.yaw);
            obs.axis_y = std::sin(obs.yaw);
        }
        nh.param<int>("dynamic_obstacles/num", count, 0);
        nh.param<float>("dynamic_obstacles/height", default_height, 10.0f);
        dyn_obs_vec.resize(std::max(0, count));
        for (int i = 0; i < count; ++i) {
            const std::string prefix = "dynamic_obstacles/dyn_obs" + std::to_string(i + 1);
            dyn_obs_t& obs = dyn_obs_vec[i];
            nh.param<float>(prefix + "/x", obs.pos[0], 0.0f);
            nh.param<float>(prefix + "/y", obs.pos[1], 0.0f);
            nh.param<float>(prefix + "/r", obs.r, 5.0f);
            nh.param<float>(prefix + "/height", obs.height, default_height);
            nh.param<float>(prefix + "/z", obs.z, obs.height / 2.0f);
            nh.param<std::string>(prefix + "/type", obs.type, "linear");
            nh.param<float>(prefix + "/velocity", obs.velocity, 0.0f);
            float direction;
            nh.param<float>(prefix + "/direction", direction, 0.0f);
            obs.direction = direction * Rad2Deg;
            nh.param<float>(prefix + "/angular_velocity", obs.angular_velocity, 0.0f);
            nh.param<float>(prefix + "/radius", obs.radius, 0.0f);
            nh.param<float>(prefix + "/center/x", obs.center[0], 0.0f);
            nh.param<float>(prefix + "/center/y", obs.center[1], 0.0f);
            obs.current_pos[0] = obs.pos[0];
            obs.current_pos[1] = obs.pos[1];
            obs.last_update_time = ros::Time::now();
        }
        odom_sub = nh.subscribe("odom", 1, &MotionPrimitives::odom_cb, this);
        cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        traj_visual_pub = nh.advertise<visualization_msgs::MarkerArray>("traj", 1);
        obs_visual_pub = nh.advertise<visualization_msgs::MarkerArray>("obs", 1);
        goal_visual_pub = nh.advertise<visualization_msgs::Marker>("goal", 1);
        planning_timer = nh.createTimer(ros::Duration(0.1), &MotionPrimitives::planningCallback, this);
    }

    ~MotionPrimitives() {
        delete state_ptr;
        delete last_pr_ptr;
    }

    std::vector<pr_t> generate_library(const s_t& s) {
        // Split both sides of the reachable height interval around the goal.
        // This includes goal-height tracking without collapsing all levels onto
        // the ground when starting close to the minimum flight altitude.
        const float low = std::max(min_altitude, s.h - mp_d * std::tan(max_descent_angle));
        const float high = std::min(max_altitude, s.h + mp_d * std::tan(max_climb_angle));
        const float center = constraint(global_target.h, low, high);
        std::vector<float> heights;
        const int half = vertical_num / 2;
        for (int j = -half; j <= half; ++j) {
            const float h = half == 0 ? center : j < 0
                ? center + (center - low) * j / half
                : center + (high - center) * j / half;
            if (heights.empty() || std::abs(h - heights.back()) > 1e-4f)
                heights.push_back(h);
        }
        std::vector<pr_t> library;
        library.reserve(mp_num * heights.size());
        for (float h : heights) {
            for (int i = 0; i < mp_num; ++i) {
                wp_t wp;
                wp.d = mp_d;
                wp.theata = mp_num == 1 ? 0.0f : -mp_theata + i * 2.0f * mp_theata / (mp_num - 1);
                wp.h = h;
                wp.V = mp_V;
                pr_t pr = generate_unlimited_agility_cmd(wp, s);
                if (tau_a > 0.1f) pr = compensate_cmd(pr, s);
                library.push_back(pr);
            }
        }
        return library;
    }

    void planningCallback(const ros::TimerEvent&) {
        if (!state_ptr) return;
        updateDynamicObstacles();
        check_target(*state_ptr);
        const std::vector<pr_t> library = generate_library(*state_ptr);
        visualization_msgs::MarkerArray markers;
        float best_cost = std::numeric_limits<float>::infinity();
        pr_t selected;
        rollout_t selected_rollout;
        int id = 1;
        for (const pr_t& pr : library) {
            // Score and display the exact command that will be sent, once.
            const rollout_t prediction = rollout_motion_primitive(*state_ptr, pr);
            const float cost = evaluate_motion_primitive(*state_ptr, pr, prediction);
            if (cost < best_cost) {
                selected = pr;
                selected_rollout = prediction;
                best_cost = cost;
            }
            auto marker = visual_motion_primitive(prediction);
            marker.ns = "traj" + std::to_string(id);
            marker.id = id++;
            markers.markers.push_back(marker);
        }
        if (best_cost >= collision_cost && last_pr_ptr) {
            const rollout_t prediction = rollout_motion_primitive(*state_ptr, *last_pr_ptr);
            const float cost = evaluate_motion_primitive(*state_ptr, *last_pr_ptr, prediction);
            if (cost < best_cost) {
                selected = *last_pr_ptr;
                selected_rollout = prediction;
                best_cost = cost;
            }
        }
        if (!std::isfinite(best_cost)) {
            wp_t recovery;
            recovery.d = mp_d;
            recovery.V = mp_V;
            recovery.h = constraint(state_ptr->h + 5.0f, min_altitude, max_altitude);
            selected = generate_unlimited_agility_cmd(recovery, *state_ptr);
            selected_rollout = rollout_motion_primitive(*state_ptr, selected);
        }
        if (best_cost >= collision_cost)
            ROS_WARN_THROTTLE(2.0, "No collision-free 3D primitive in the current local horizon");
        auto optimal = visual_motion_primitive(selected_rollout);
        optimal.ns = "traj_optimal";
        optimal.id = 1;
        optimal.color.r = 1.0f;
        optimal.color.g = 0.0f;
        optimal.color.b = best_cost >= collision_cost ? 1.0f : 0.0f;
        optimal.color.a = 1.0f;
        optimal.scale.x = 0.45f;
        markers.markers.push_back(optimal);
        // Remove old candidate markers if clipping/deduplication shrinks a library.
        for (int old_id = id; old_id <= previous_marker_count; ++old_id) {
            visualization_msgs::Marker removed;
            removed.ns = "traj" + std::to_string(old_id);
            removed.id = old_id;
            removed.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(removed);
        }
        previous_marker_count = id - 1;
        traj_visual_pub.publish(markers);
        obs_visual_pub.publish(collision_visual());
        goal_visual_pub.publish(goal_visual());
        if (!last_pr_ptr) last_pr_ptr = new pr_t;
        *last_pr_ptr = selected;
        geometry_msgs::Twist command;
        command.linear.x = selected.w_roll_c;
        command.linear.y = selected.attack_c;
        command.linear.z = selected.T_c;
        cmd_pub.publish(command);
    }

    void updateDynamicObstacles() {
        const ros::Time now = ros::Time::now();
        for (auto& obs : dyn_obs_vec) {
            const float dt = (now - obs.last_update_time).toSec();
            obs.last_update_time = now;
            if (dt <= 0) continue;
            if (obs.type == "linear") {
                obs.current_pos[0] += obs.velocity * std::cos(obs.direction) * dt;
                obs.current_pos[1] += obs.velocity * std::sin(obs.direction) * dt;
            } else if (obs.type == "circular") {
                obs.angle = wrap_yaw(obs.angle + obs.angular_velocity * dt);
                obs.current_pos[0] = obs.center[0] + obs.radius * std::cos(obs.angle);
                obs.current_pos[1] = obs.center[1] + obs.radius * std::sin(obs.angle);
            }
        }
    }

    pr_t compensate_cmd(const pr_t& pr, const s_t& s) {
        const float nu = s.d_sigma * tau_a / 2.0f;
        const float delay_distance = s.V * tau_a;
        const float distance = std::sqrt(std::max(0.01f, pr.wp.d * pr.wp.d +
            delay_distance * delay_distance - 2.0f * pr.wp.d * delay_distance * std::cos(pr.wp.theata - nu)));
        float angle = std::acos(constraint((delay_distance - pr.wp.d * std::cos(pr.wp.theata - nu)) / distance, -1.0f, 1.0f));
        if (pr.wp.theata - nu < 0) angle = -angle;
        wp_t wp = pr.wp;
        wp.d = distance;
        wp.theata = constraint(wrap_yaw(M_PI - angle - nu), -mp_theata, mp_theata);
        pr_t result = generate_unlimited_agility_cmd(wp, s);
        result.wp = pr.wp;
        result.t_way = pr.t_way;
        return result;
    }

    void check_target(const s_t& s) {
        if (std::hypot(s.pos[0] - global_target.pos[0], s.pos[1] - global_target.pos[1]) <= mp_d + 0.15f * mp_V &&
            std::abs(s.h - global_target.h) <= altitude_tolerance) {
            wp_i = (wp_i + 1) % wp_n;
            global_target = wp_vec[wp_i];
        }
    }

    void odom_cb(const nav_msgs::Odometry& msg) {
        s_t s;
        s.V = msg.twist.twist.linear.x;
        s.gamma = msg.twist.twist.linear.y;
        s.sigma = msg.twist.twist.linear.z;
        s.T = msg.twist.twist.angular.x;
        s.d_sigma = msg.twist.twist.angular.y;
        s.pos[0] = msg.pose.pose.position.x;
        s.pos[1] = msg.pose.pose.position.y;
        s.h = msg.pose.pose.position.z;
        const auto& q = msg.pose.pose.orientation;
        s.q[0] = q.w; s.q[1] = q.x; s.q[2] = q.y; s.q[3] = q.z;
        if (!std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
            q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z < 1e-8) return;
        tf::Quaternion orientation(q.x, q.y, q.z, q.w);
        orientation.normalize();
        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        s.roll = -roll;
        s.attack = -pitch - s.gamma;
        if (!finite_state(s)) {
            ROS_WARN_THROTTLE(2.0, "Ignoring non-finite flight odometry");
            return;
        }
        if (!state_ptr) state_ptr = new s_t;
        *state_ptr = s;
    }

    float C_D(float attack) const { return CD_k1 * attack * attack + CD_k2; }
    float C_L(float attack) const { return CL_k1 * std::sin(attack) + CL_k2; }
    static float wrap_yaw(float angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
    static float abs_err_angle(float a, float b) { return std::abs(wrap_yaw(a - b)); }
    static float constraint(float x, float low, float high) { return std::max(low, std::min(x, high)); }

    pr_t generate_unlimited_agility_cmd(const wp_t& wp, const s_t& s) {
        pr_t result;
        const float speed = std::max(min_speed, s.V);
        const float distance = std::max(0.1f, wp.d);
        const float gain = 2.0f * speed / distance;
        const float gamma_c = std::atan2(wp.h - s.h, distance);
        const float gamma_dot = gain * (gamma_c - s.gamma);
        const float y = 2.0f * speed * speed * std::sin(wp.theata) * std::pow(std::cos(s.gamma), 2);
        const float x = distance * (gravity * std::cos(s.gamma) + speed * gamma_dot);
        result.w_roll_c = constraint(std::atan2(y, x), -mp_max_roll, mp_max_roll);
        const float lift_scale = k * speed * speed * std::max(0.1f, std::cos(result.w_roll_c));
        const float a = (gamma_dot * speed + gravity * std::cos(gamma_c)) / lift_scale;
        const float b = s.T / lift_scale;
        const float denominator = std::max(1e-5f, CL_k1 + b);
        result.attack_c = constraint(std::asin(constraint((a - CL_k2) / denominator, -1.0f, 1.0f)),
                                     mp_min_attack, mp_max_attack);
        result.T_c = constraint((k * speed * speed * C_D(result.attack_c) + gravity * std::sin(gamma_c)) /
            std::max(0.1f, std::cos(result.attack_c)) + mp_k_T * gain * (wp.V - speed), 0.0f, max_thrust);
        result.t_way = horizon_time;
        result.wp = wp;
        return result;
    }

    bool collision_check(float x, float y, float z, float prediction_time = 0.0f) const {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
            z < min_altitude || z > max_altitude) return true;
        for (const auto& obs : static_obs_vec) {
            if (inside_cylinder(x, y, z, obs.pos[0], obs.pos[1], obs.z, obs.r, obs.height))
                return true;
        }
        for (const auto& obs : sphere_obs_vec) {
            // Inflate in every direction equally; a floating sphere has no
            // cylindrical footprint extending to its top and bottom corners.
            const float radius = obs.r + std::max(0.2f * obs.r, vertical_clearance);
            const float dx = x - obs.x, dy = y - obs.y, dz = z - obs.z;
            if (dx * dx + dy * dy + dz * dz <= radius * radius) return true;
        }
        for (const auto& obs : horizontal_obs_vec) {
            const float dx = x - obs.x, dy = y - obs.y, dz = z - obs.z;
            const float axial = dx * obs.axis_x + dy * obs.axis_y;
            const float transverse = -dx * obs.axis_y + dy * obs.axis_x;
            const float margin = std::max(0.2f * obs.r, vertical_clearance);
            const float radius = obs.r + margin;
            // Transform into the cylinder's frame: finite flat caps along its
            // horizontal axis, circular cross-section in transverse/height.
            if (std::abs(axial) <= obs.length / 2.0f + margin &&
                transverse * transverse + dz * dz <= radius * radius) return true;
        }
        for (const auto& obs : dyn_obs_vec) {
            float ox = obs.current_pos[0], oy = obs.current_pos[1];
            if (prediction_time > 0 && obs.type == "linear") {
                ox += obs.velocity * std::cos(obs.direction) * prediction_time;
                oy += obs.velocity * std::sin(obs.direction) * prediction_time;
            } else if (prediction_time > 0 && obs.type == "circular") {
                const float angle = obs.angle + obs.angular_velocity * prediction_time;
                ox = obs.center[0] + obs.radius * std::cos(angle);
                oy = obs.center[1] + obs.radius * std::sin(angle);
            }
            if (inside_cylinder(x, y, z, ox, oy, obs.z, obs.r, obs.height)) return true;
        }
        return false;
    }

    rollout_t rollout_motion_primitive(const s_t& initial, const pr_t& pr) const {
        rollout_t result;
        result.states.push_back(initial);
        s_t s = initial;
        if (!finite_state(s) || !std::isfinite(pr.T_c) || !std::isfinite(pr.attack_c) ||
            !std::isfinite(pr.w_roll_c) || !std::isfinite(pr.t_way) || pr.t_way <= 0) {
            result.valid = false;
            return result;
        }
        result.collision = collision_check(s.pos[0], s.pos[1], s.h);
        for (float t = 0; t < pr.t_way - 1e-6f;) {
            const float dt = std::min(integration_dt, pr.t_way - t);
            const float speed = std::max(1.0f, s.V);
            const float lift = s.T * std::sin(s.attack) / speed + k * speed * C_L(s.attack);
            const float dV = s.T * std::cos(s.attack) - k * speed * speed * C_D(s.attack) - gravity * std::sin(s.gamma);
            const float dgamma = lift * std::cos(s.roll) - gravity * std::cos(s.gamma) / speed;
            const float dsigma = lift * std::sin(s.roll) / std::max(0.1f, std::cos(s.gamma));
            s.pos[0] += speed * std::cos(s.gamma) * std::cos(s.sigma) * dt;
            s.pos[1] += speed * std::cos(s.gamma) * std::sin(s.sigma) * dt;
            s.h += speed * std::sin(s.gamma) * dt;
            s.V += dV * dt;
            s.gamma += dgamma * dt;
            s.sigma = wrap_yaw(s.sigma + dsigma * dt);
            s.d_sigma = dsigma;
            // Match the three first-order actuators in the existing Gazebo plugin.
            s.attack += (pr.attack_c - s.attack) * dt;
            s.roll += 2.0f * (pr.w_roll_c - s.roll) * dt;
            s.T += 10.0f * (pr.T_c - s.T) * dt;
            t += dt;
            result.states.push_back(s);
            if (!finite_state(s) || s.V < min_speed || s.V > max_speed || std::abs(s.gamma) > 80.0f * Rad2Deg) {
                result.valid = false;
                break;
            }
            if (collision_check(s.pos[0], s.pos[1], s.h, t)) result.collision = true;
        }
        return result;
    }

    float evaluate_motion_primitive(const s_t& s, const pr_t& pr) const {
        return evaluate_motion_primitive(s, pr, rollout_motion_primitive(s, pr));
    }

    float evaluate_motion_primitive(const s_t&, const pr_t& pr, const rollout_t& prediction) const {
        if (!prediction.valid || prediction.states.empty()) return std::numeric_limits<float>::infinity();
        const s_t& end = prediction.states.back();
        const float ex = global_target.pos[0] - end.pos[0], ey = global_target.pos[1] - end.pos[1];
        const float ez = global_target.h - end.h;
        const float horizontal_distance = std::hypot(ex, ey);
        float cost = distance_weight * std::sqrt(ex*ex + ey*ey + ez*ez) + altitude_weight * std::abs(ez);
        if (horizontal_distance > 1.0f) cost += heading_weight * abs_err_angle(std::atan2(ey, ex), end.sigma);
        if (last_pr_ptr) {
            cost += 0.1f * std::abs(pr.attack_c - last_pr_ptr->attack_c);
            cost += 1.0f * std::abs(pr.w_roll_c - last_pr_ptr->w_roll_c);
            cost += 0.05f * std::abs(pr.T_c - last_pr_ptr->T_c);
        }
        if (prediction.collision) cost += collision_cost;
        return cost;
    }

    visualization_msgs::Marker visual_motion_primitive(const s_t& s, const pr_t& pr) const {
        return visual_motion_primitive(rollout_motion_primitive(s, pr));
    }

    visualization_msgs::Marker visual_motion_primitive(const rollout_t& prediction) const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.12;
        marker.color.g = 1;
        marker.color.a = prediction.valid && !prediction.collision ? 0.65 : 0.15;
        for (const auto& s : prediction.states) {
            if (!finite_state(s)) break;
            geometry_msgs::Point p;
            p.x = s.pos[0]; p.y = s.pos[1]; p.z = s.h;
            marker.points.push_back(p);
        }
        return marker;
    }

    visualization_msgs::Marker goal_visual() const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "goal";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = global_target.pos[0];
        marker.pose.position.y = global_target.pos[1];
        marker.pose.position.z = global_target.h;
        marker.pose.orientation.w = 1;
        marker.scale.x = marker.scale.y = marker.scale.z = 5;
        marker.color.g = marker.color.b = marker.color.a = 1;
        return marker;
    }

    visualization_msgs::MarkerArray collision_visual() const {
        visualization_msgs::MarkerArray result;
        int id = 1;
        for (size_t i = 0; i < static_obs_vec.size(); ++i) {
            const auto& obs = static_obs_vec[i];
            auto marker = cylinder_marker("static_obs" + std::to_string(i + 1), id++, obs.pos[0], obs.pos[1], obs.z, obs.r, obs.height);
            marker.color.b = 1;
            result.markers.push_back(marker);
        }
        for (size_t i = 0; i < dyn_obs_vec.size(); ++i) {
            const auto& obs = dyn_obs_vec[i];
            auto marker = cylinder_marker("dyn_obs" + std::to_string(i + 1), id++, obs.current_pos[0], obs.current_pos[1], obs.z, obs.r, obs.height);
            marker.color.r = 1;
            result.markers.push_back(marker);
        }
        for (size_t i = 0; i < sphere_obs_vec.size(); ++i) {
            const auto& obs = sphere_obs_vec[i];
            auto marker = cylinder_marker("sphere_obs" + std::to_string(i + 1), id++,
                                          obs.x, obs.y, obs.z, obs.r, 2 * obs.r);
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.color.b = 1.0f;
            result.markers.push_back(marker);
        }
        for (size_t i = 0; i < horizontal_obs_vec.size(); ++i) {
            const auto& obs = horizontal_obs_vec[i];
            auto marker = cylinder_marker("horizontal_obs" + std::to_string(i + 1), id++,
                                          obs.x, obs.y, obs.z, obs.r, obs.length);
            marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI / 2.0, obs.yaw);
            marker.color.b = 1.0f;
            result.markers.push_back(marker);
        }
        return result;
    }

private:
    void load_angle(const std::string& name, float& value, float degrees) {
        nh.param<float>("motion_primitive/" + name, value, degrees);
        value *= Rad2Deg;
    }
    static bool finite_state(const s_t& s) {
        return std::isfinite(s.V) && std::isfinite(s.gamma) && std::isfinite(s.sigma) &&
            std::isfinite(s.T) && std::isfinite(s.h) && std::isfinite(s.pos[0]) &&
            std::isfinite(s.pos[1]) && std::isfinite(s.roll) && std::isfinite(s.attack) && std::isfinite(s.d_sigma);
    }
    bool inside_cylinder(float x, float y, float z, float ox, float oy, float center_z, float radius, float height) const {
        return std::abs(z - center_z) <= height / 2.0f + vertical_clearance &&
            std::hypot(x - ox, y - oy) <= radius * 1.2f;
    }
    visualization_msgs::Marker cylinder_marker(const std::string& name, int id, float x, float y, float z, float radius, float height) const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = name;
        marker.id = id;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x; marker.pose.position.y = y; marker.pose.position.z = z;
        marker.pose.orientation.w = 1;
        marker.scale.x = marker.scale.y = 2 * radius;
        marker.scale.z = height;
        marker.color.a = 0.5;
        return marker;
    }

    const float gravity = 9.81f, k = 1.86f, collision_cost = 10000.0f;
    float mp_d, mp_V, mp_theata, mp_max_roll, mp_max_attack, mp_min_attack, mp_k_T;
    float horizon_time, max_climb_angle, max_descent_angle, min_altitude, max_altitude;
    float integration_dt, min_speed, max_speed, max_thrust, vertical_clearance;
    float altitude_weight, distance_weight, heading_weight, altitude_tolerance;
    float CL_k1, CL_k2, CD_k1, CD_k2, tau_a;
    int mp_num, vertical_num, wp_i = 0, wp_n, previous_marker_count = 0;
    s_t* state_ptr = nullptr;
    pr_t* last_pr_ptr = nullptr;
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Publisher cmd_pub, traj_visual_pub, obs_visual_pub, goal_visual_pub;
    ros::Timer planning_timer;
    target_t global_target;
    std::vector<target_t> wp_vec;
    std::vector<obs_t> static_obs_vec;
    std::vector<sphere_obstacle> sphere_obs_vec;
    std::vector<horizontal_obstacle> horizontal_obs_vec;
    std::vector<dyn_obs_t> dyn_obs_vec;
};
#endif
