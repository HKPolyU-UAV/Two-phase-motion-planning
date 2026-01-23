#ifndef MOTION_PRIMITIVES_H
#define MOTION_PRIMITIVES_H
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <cmath>
#define Rad2Deg 0.0174533

class MotionPrimitives {
    public:
        MotionPrimitives(ros::NodeHandle _nh): nh(_nh), collision_cost(10000.0f) {
            gravity = 9.81f;
            k = 1.86f;
            state_ptr = nullptr;
            last_pr_ptr = nullptr;
            odom_sub = nh.subscribe("odom", 1,&MotionPrimitives::odom_cb, this);
            cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            traj_visual_pub = nh.advertise<visualization_msgs::MarkerArray>("traj", 1);
            obs_visual_pub = nh.advertise<visualization_msgs::MarkerArray>("obs", 1);
            goal_visual_pub = nh.advertise<visualization_msgs::Marker>("goal", 1);
            planning_timer = nh.createTimer(ros::Duration(0.1), &MotionPrimitives::planningCallback, this);
            nh.param<float>("model/CL/k1", CL_k1, 0.03f);
            nh.param<float>("model/CL/k2", CL_k2, 0.028f);
            nh.param<float>("model/CD/k1", CD_k1, 0.001f);
            nh.param<float>("model/CD/k2", CD_k2, 0.002f);
            float mp_horizon_time;
            nh.param<float>("motion_primitive/horizon_time", mp_horizon_time, 3.0f);
            nh.param<float>("motion_primitive/V", mp_V, 13.5f);
            mp_d = mp_horizon_time * mp_V;
            float tmp_degree;
            nh.param<float>("motion_primitive/theata", tmp_degree, 45.0f);
            mp_theata = tmp_degree * Rad2Deg;
            nh.param<int>("motion_primitive/num", mp_num, 29);
            nh.param<float>("motion_primitive/max_roll_angle", tmp_degree, 35.0f);
            mp_max_roll = tmp_degree * Rad2Deg;
            nh.param<float>("motion_primitive/max_attack_angle", tmp_degree, 25.0f);
            mp_max_attack = tmp_degree * Rad2Deg;
            nh.param<float>("motion_primitive/min_attack_angle", tmp_degree, -15.0f);
            mp_min_attack = tmp_degree * Rad2Deg;
            nh.param<float>("motion_primitive/compensate_tau", tau_a, 0.2f);
            nh.param<float>("motion_primitive/K_T", mp_k_T, 1.2f);

            nh.param<int>("waypoints/num", wp_n, 4);
            wp_vec.resize(wp_n);
            for (int i = 0; i < wp_n; i++) {
                nh.param<float>("waypoints/wp"+std::to_string(i+1)+"/x", wp_vec[i].pos[0], 300.0f);
                nh.param<float>("waypoints/wp"+std::to_string(i+1)+"/y", wp_vec[i].pos[1], 0.0f);
                nh.param<float>("waypoints/wp"+std::to_string(i+1)+"/h", wp_vec[i].h , 2.5f);
            }
            wp_i = 0;
            global_target = wp_vec[wp_i];

            int num_obs;
            nh.param<int>("obstacles/num", num_obs, 4);
            obs_vec.resize(num_obs);
            for (int i = 0; i < num_obs; i++) {
                nh.param<float>("obstacles/obs"+std::to_string(i+1)+"/x", obs_vec[i].pos[0], 110.0f);
                nh.param<float>("obstacles/obs"+std::to_string(i+1)+"/y", obs_vec[i].pos[1], 10.0f);
                nh.param<float>("obstacles/obs"+std::to_string(i+1)+"/r", obs_vec[i].r ,10.0f);
            }
            planning_timer.start();
        }

        ~MotionPrimitives() {

        }

        typedef struct waypoint {
            float d;
            float theata;
            float h;
            float V;
        } wp_t;

        typedef struct primitive {
            float w_roll_c;
            float attack_c;
            float T_c;
            float t_way;
            wp_t wp;
            void print() {
                printf("pr info: \n");
                printf("roll: %.2f  attack: %.2f T: %.2f\n", w_roll_c, attack_c, T_c);
            }
        } pr_t;

        typedef struct state {
            float V;
            float gamma;
            float sigma;
            float d_sigma;
            float T;
            float h;
            float pos[2]; // x y
            float q[4]; // w x y z
        } s_t;

        typedef struct target {
            float pos[2];
            float h;
        } target_t;
        
        typedef struct obstacle {
            float pos[2];
            float r;
        } obs_t;

        void planningCallback(const ros::TimerEvent &e) {
            if (state_ptr == nullptr)
                return;
            check_target(*state_ptr);
            // if (std::abs(state_ptr->pos[0] - 30.0f) < 5.0f && std::abs()

            float cost = std::numeric_limits<float>::max();
            pr_t final_pr;
            visualization_msgs::MarkerArray traj_visual_array;
            traj_visual_array.markers.resize(0);
            int traj_id = 1;
            for (int i = 0; i < mp_num; i++) {
                float theata = -mp_theata + i * (2.0f * mp_theata) / (mp_num - 1);
                wp_t wp;
                wp.d = mp_d;
                wp.theata = theata;
                wp.h = global_target.h;
                wp.V = mp_V;
                pr_t pr = generate_unlimited_agility_cmd(wp, *state_ptr);
                float pr_cost = evaluate_motion_primitive(*state_ptr, pr);
                if (pr_cost < cost) {
                    final_pr = pr;
                    cost = pr_cost;
                }
                visualization_msgs::Marker traj_marker = visual_motion_primitive(*state_ptr, pr);
                traj_marker.ns = "traj" + std::to_string(traj_id);
                traj_marker.id = traj_id;
                traj_visual_array.markers.push_back(traj_marker);
                traj_id ++;
            }

            bool no_valid_pr = false;
            if (cost > collision_cost - 100.0f && last_pr_ptr != nullptr) {
                final_pr = *last_pr_ptr;
                no_valid_pr = true;
            }

            visualization_msgs::Marker traj_marker = visual_motion_primitive(*state_ptr, final_pr);
            traj_marker.ns = "traj_optimal";
            traj_marker.id = traj_id;
            if (no_valid_pr) {
                traj_marker.color.b = 1.0f;
                traj_marker.color.g = 0.0f;
            } else {
                traj_marker.color.r = 1.0f;
                traj_marker.color.g = 0.0f;
            }
            traj_marker.scale.x = 0.7f;
            traj_marker.scale.y = 0.7f;
            traj_visual_array.markers.push_back(traj_marker);
            traj_visual_pub.publish(traj_visual_array);
            obs_visual_pub.publish(collision_visual());
            goal_visual_pub.publish(goal_visual());

            if (last_pr_ptr == nullptr) {
                last_pr_ptr = new pr_t;
            }

            /* compensate system delay */
            if (tau_a > 0.1f)
                final_pr = compensate_cmd(final_pr, *state_ptr, global_target.h);

            *last_pr_ptr = final_pr;
            final_pr.print();

            geometry_msgs::Twist cmd_msg;
            cmd_msg.linear.x = final_pr.w_roll_c;
            cmd_msg.linear.y = final_pr.attack_c;
            cmd_msg.linear.z = final_pr.T_c;
            cmd_pub.publish(cmd_msg);
        }

        pr_t compensate_cmd(const pr_t& pr, const s_t& s, const float& h) {
            float nu = s.d_sigma * tau_a / 2.0f;
            float d_new = std::sqrt( pr.wp.d * pr.wp.d + (s.V * tau_a)*(s.V * tau_a) - 2.0f * pr.wp.d * s.V * tau_a * std::cos(pr.wp.theata - nu));
            float tmp_cos_nu_a = (s.V*tau_a - pr.wp.d * std::cos(pr.wp.theata - nu)) / d_new;
            // printf("cos nu: %.2f\n", tmp_cos_nu_a);
            tmp_cos_nu_a = constraint(tmp_cos_nu_a, -1.0f + 1e-6f, 1.0f - 1e-6f); // to avoid nan result
            float nu_a = std::abs(std::acos(tmp_cos_nu_a));
            if (pr.wp.theata - nu < 0.0f) {
                nu_a = -nu_a;
            }
            // printf("nu: %.2f, d_new: %.2f, nu_a: %.2f\n", nu, d_new, nu_a);
            wp_t wp;
            wp.d = d_new;
            wp.V = mp_V;
            wp.h = h;
            wp.theata = M_PI - nu_a - nu;
            wp.theata = constraint(wrap_yaw(wp.theata), -mp_theata, mp_theata);
            pr_t res = generate_unlimited_agility_cmd(wp, s);
            res.wp = pr.wp;
            return res;
        }

        void check_target(const s_t& state) {
            int next_wp_i = wp_i+1 > wp_n-1 ? 0: wp_i+1;
            if (std::abs(state.pos[0] - global_target.pos[0]) < mp_d + 0.15f* mp_V &&
                std::abs(state.pos[1] - global_target.pos[1]) < mp_d + 0.15f* mp_V) {
                global_target = wp_vec[next_wp_i];
                wp_i = next_wp_i;
                std::cout << " wp " << wp_i << " !!!!!!!!" << std::endl;
            }
        }
        
        void odom_cb(const nav_msgs::Odometry &msg) {
            if (state_ptr == nullptr) {
                state_ptr = new s_t;
            }
            state_ptr->V = msg.twist.twist.linear.x;
            state_ptr->gamma = msg.twist.twist.linear.y;
            state_ptr->sigma = msg.twist.twist.linear.z;
            state_ptr->T = msg.twist.twist.angular.x;
            state_ptr->d_sigma = msg.twist.twist.angular.y;
            state_ptr->h = msg.pose.pose.position.z;
            state_ptr->pos[0] = msg.pose.pose.position.x;
            state_ptr->pos[1] = msg.pose.pose.position.y;
            state_ptr->q[0] = msg.pose.pose.orientation.w;
            state_ptr->q[1] = msg.pose.pose.orientation.x;
            state_ptr->q[2] = msg.pose.pose.orientation.y;
            state_ptr->q[3] = msg.pose.pose.orientation.z;


// #define test
#ifdef test
            wp_t wp;
            wp.d = 30.0f;
            wp.theata = -0.2f;
            wp.h = 2.4f;//msg.pose.pose.position.z;
            wp.V = mp_V;
            pr_t test_res = generate_unlimited_agility_cmd(wp, *state_ptr);
            test_res.print();
            geometry_msgs::Twist cmd_msg;
            cmd_msg.linear.x = test_res.w_roll_c;
            cmd_msg.linear.y = test_res.attack_c;
            cmd_msg.linear.z = test_res.T_c;
            cmd_pub.publish(cmd_msg);
            float cost = evaluate_motion_primitive(*state_ptr, test_res);
#endif
        }

        float C_D(float attack) {
            // return 0.01f*attack*attack + 0.02f;
            return CD_k1*attack*attack + CD_k2;
        }
        float C_L(float attack) {
            // return 0.1f*sinf(attack) + 0.42f;
            return CL_k1*sinf(attack) + CL_k2;
        }
        float wrap_yaw(float a) {
            return fmod(a + M_PI,2.0f*M_PI) - M_PI;
        }
        float abs_err_angle(float a, float b) {
            if (std::abs(a-b) > M_PI) {
                return 2*M_PI - std::abs(a-b);
            }
            return std::abs(a-b);
        }
        float constraint(float x, float a, float b) {
            return std::max(std::min(x, b), a);
        }

        pr_t generate_unlimited_agility_cmd(const wp_t &wp, const s_t &s) {
            pr_t res;
            float t_way = wp.d / s.V;
            float k_T = 2.0f/t_way;
            float gamma_c = std::atan2(wp.h - s.h, wp.d);
            float gamma_dot_des = k_T * (gamma_c - s.gamma);
            float y = 2.0f*s.V*s.V*std::sin(wp.theata)*std::cos(s.gamma)*std::cos(s.gamma);
            float x = wp.d * (gravity*std::cos(s.gamma) + s.V*gamma_dot_des);
            res.w_roll_c = std::atan2(y,x);
            res.w_roll_c = constraint(res.w_roll_c, -mp_max_roll, mp_max_roll);
            float a = (gamma_dot_des*s.V + gravity*std::cos(gamma_c)) / (k*s.V*s.V*std::cos(res.w_roll_c));
            float b = s.T / (k*s.V*s.V*std::cos(res.w_roll_c));
            float tmp_sin_res = constraint((a - CL_k2)/(CL_k1 + b), -1.0f + 1e-6f, 1.0f - 1e-6f);
            res.attack_c = std::asin(tmp_sin_res);
            res.attack_c = constraint(res.attack_c, mp_min_attack, mp_max_attack);
            res.T_c = (k*s.V*s.V*C_D(res.attack_c) + gravity * std::sin(gamma_c)) / std::cos(res.attack_c)
                            + mp_k_T*k_T*(wp.V - s.V);
            res.t_way = t_way;
            res.wp = wp;
            return res;
        }

        visualization_msgs::Marker goal_visual() {
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "world";
            marker.ns = "goal";
            marker.id = 1;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = global_target.pos[0];
            marker.pose.position.y = global_target.pos[1];
            marker.pose.position.z = global_target.h;
            marker.pose.orientation.w = 1.0f;
            marker.pose.orientation.x = 0.0f;
            marker.pose.orientation.y = 0.0f;
            marker.pose.orientation.z = 0.0f;
            marker.scale.x = 5.0;
            marker.scale.y = 5.0;
            marker.scale.z = 5.0;
            marker.color.a = 1.0f;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f;
            return marker;
        }

        visualization_msgs::MarkerArray collision_visual() {
            visualization_msgs::MarkerArray marker_array;
            marker_array.markers.resize(0);
            for (int i = 0 ; i < obs_vec.size(); i++) {
                visualization_msgs::Marker marker;
                marker.header.stamp = ros::Time::now();
                marker.header.frame_id = "world";
                marker.ns = "obs" + std::to_string(i+1);
                marker.id = i+1;
                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = obs_vec[i].pos[0];
                marker.pose.position.y = obs_vec[i].pos[1];
                marker.pose.position.z = 5.0f;
                marker.pose.orientation.w = 1.0f;
                marker.pose.orientation.x = 0.0f;
                marker.pose.orientation.y = 0.0f;
                marker.pose.orientation.z = 0.0f;
                marker.scale.x = 2.0f*obs_vec[i].r;
                marker.scale.y = 2.0f*obs_vec[i].r;
                marker.scale.z = 10.0f;
                marker.color.a = 0.5f;
                marker.color.r = 0.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;
                marker_array.markers.push_back(marker);
            }
            return marker_array;
        }

        bool collision_check(float x, float y) {
            for (int i = 0; i < obs_vec.size(); i++) {
                if (std::abs( x - obs_vec[i].pos[0] ) < obs_vec[i].r*1.2f
                    && std::abs( y - obs_vec[i].pos[1] ) < obs_vec[i].r*1.2f) {
                    return true;
                }
            }
            return false;
        }

        visualization_msgs::Marker visual_motion_primitive(const s_t &s, const pr_t &pr) {
            float x = s.pos[0];
            float y = s.pos[1];
            float z = s.h;
            float V = s.V;
            float gamma = s.gamma;
            float sigma = s.sigma;
            float dt = 0.01f;
            float cost = 0.0f;
            visualization_msgs::Marker traj_marker;
            traj_marker.header.stamp = ros::Time::now();
            traj_marker.header.frame_id = "world";
            traj_marker.ns = "trajectory";
            traj_marker.id = 1;
            traj_marker.type = visualization_msgs::Marker::POINTS;
            traj_marker.action = visualization_msgs::Marker::ADD;
            traj_marker.pose.position.x = 0.0f;
            traj_marker.pose.position.y = 0.0f;
            traj_marker.pose.position.z = 0.0f;
            traj_marker.pose.orientation.w = 1.0f;
            traj_marker.pose.orientation.x = 0.0f;
            traj_marker.pose.orientation.y = 0.0f;
            traj_marker.pose.orientation.z = 0.0f;
            traj_marker.scale.x = 0.2f;
            traj_marker.scale.y = 0.2f;
            traj_marker.color.a = 1.0f;
            traj_marker.color.r = 0.0f;
            traj_marker.color.g = 1.0f;
            traj_marker.color.b = 0.0f;
            traj_marker.points.resize(0);
            for (float t = 0; t < pr.t_way; t += dt) {
                float dx = V*std::cos(gamma)*std::cos(sigma);
                float dy = V*std::cos(gamma)*std::sin(sigma);
                float dh = V*std::sin(gamma);
                float dV = (pr.T_c*std::cos(pr.attack_c) - k*V*V*C_D(pr.attack_c)) - gravity*std::sin(gamma);
                float d_gamma = (pr.T_c*std::sin(pr.attack_c)/V + k*V*C_L(pr.attack_c))*std::cos(pr.w_roll_c)
                         - gravity*std::cos(gamma)/V;
                float d_sigma = (pr.T_c*std::sin(pr.attack_c)/V + k*V*C_L(pr.attack_c))*std::sin(pr.w_roll_c)/std::cos(gamma);
                x += dx*dt;
                y += dy*dt;
                z += dh*dt;
                V += dV*dt;
                gamma += d_gamma*dt;
                sigma += d_sigma*dt;
                sigma = wrap_yaw(sigma);
                geometry_msgs::Point _p;
                _p.x = x;
                _p.y = y;
                _p.z = z;
                traj_marker.points.push_back(_p);
            }
            return traj_marker;
        }

        float evaluate_motion_primitive(const s_t &s, const pr_t &pr) {
            float x = s.pos[0];
            float y = s.pos[1];
            float z = s.h;
            float V = s.V;
            float gamma = s.gamma;
            float sigma = s.sigma;
            float dt = 0.01f;
            float cost = 0.0f;

            bool collision = false;
            for (float t = 0; t < pr.t_way; t += dt) {
                float dx = V*std::cos(gamma)*std::cos(sigma);
                float dy = V*std::cos(gamma)*std::sin(sigma);
                float dh = V*std::sin(gamma);
                float dV = (pr.T_c*std::cos(pr.attack_c) - k*V*V*C_D(pr.attack_c)) - gravity*std::sin(gamma);
                float d_gamma = (pr.T_c*std::sin(pr.attack_c)/V + k*V*C_L(pr.attack_c))*std::cos(pr.w_roll_c)
                         - gravity*std::cos(gamma)/V;
                float d_sigma = (pr.T_c*std::sin(pr.attack_c)/V + k*V*C_L(pr.attack_c))*std::sin(pr.w_roll_c)/std::cos(gamma);
                x += dx*dt;
                y += dy*dt;
                z += dh*dt;
                V += dV*dt;
                if (!collision)
                    collision = collision_check(x, y);
                gamma += d_gamma*dt;
                sigma += d_sigma*dt;
                sigma = wrap_yaw(sigma);

            }

            /* calculate terminal cost */
            float ex = global_target.pos[0] - x;
            float ey = global_target.pos[1] - y;
            float dir_angle = 0.0f;
            float l_t_s = std::sqrt(ex*ex + ey*ey);
            if (l_t_s > 100.0f) {
                ex = ex/l_t_s*10.0f;
                ey = ey/l_t_s*10.0f;
            }
            if (l_t_s > 1.0f) {
                dir_angle = std::atan2(ey, ex);
            }
            cost += 3.0f*abs_err_angle(dir_angle, sigma);//(std::abs(ex) + std::abs(ey));

            if (last_pr_ptr != nullptr) {
                cost += 0.1f * std::abs(pr.attack_c - last_pr_ptr->attack_c);
                cost += 1.0f * std::abs(pr.w_roll_c - last_pr_ptr->w_roll_c);
                cost += 0.5f * std::abs(pr.T_c - last_pr_ptr->T_c);
            }
            if (collision)
                cost = collision_cost;
            return cost;
        }

    private:
        float gravity;
        float k;
        float mp_d, mp_V, mp_theata, mp_max_roll, mp_max_attack, mp_min_attack, mp_k_T;
        int mp_num;
        float CL_k1, CL_k2, CD_k1, CD_k2;
        float tau_a;
        float collision_cost;
        s_t* state_ptr;
        pr_t* last_pr_ptr;
        ros::NodeHandle nh;
        ros::Subscriber odom_sub;
        ros::Publisher cmd_pub, traj_visual_pub, obs_visual_pub, goal_visual_pub, model_visual_pub;
        ros::Timer planning_timer;
        target_t global_target;
        std::vector<target_t> wp_vec;
        std::vector<obs_t> obs_vec;
        int wp_i, wp_n;
};

#endif
