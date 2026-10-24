#include <gtest/gtest.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "../src/motion_primitives.h"

namespace {

struct Point3 {
  double x, y, z;
};

Point3 difference(const Point3 &a, const Point3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

double dot(const Point3 &a, const Point3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Point3 cross(const Point3 &a, const Point3 &b) {
  return {a.y * b.z - a.z * b.y,
          a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}

class MotionPrimitives3D : public ::testing::Test {
 protected:
  void SetUp() override {
    static unsigned int instance = 0;
    ns_ = "/motion_primitives_3d_test/case_" + std::to_string(++instance);
    nh_.reset(new ros::NodeHandle(ns_));

    nh_->setParam("motion_primitive/horizon_time", 3.0);
    nh_->setParam("motion_primitive/V", 15.5);
    nh_->setParam("motion_primitive/theata", 45.0);
    nh_->setParam("motion_primitive/num", 29);
    nh_->setParam("motion_primitive/vertical_num", 7);
    nh_->setParam("motion_primitive/max_climb_angle", 20.0);
    nh_->setParam("motion_primitive/max_descent_angle", 20.0);
    nh_->setParam("motion_primitive/min_altitude", 1.0);
    nh_->setParam("motion_primitive/max_altitude", 80.0);
    nh_->setParam("motion_primitive/integration_dt", 0.02);
    nh_->setParam("motion_primitive/min_speed", 3.0);
    nh_->setParam("motion_primitive/max_speed", 45.0);
    nh_->setParam("motion_primitive/max_thrust", 20.0);
    nh_->setParam("motion_primitive/max_roll_angle", 35.0);
    nh_->setParam("motion_primitive/max_attack_angle", 25.0);
    nh_->setParam("motion_primitive/min_attack_angle", -20.0);
    nh_->setParam("motion_primitive/compensate_tau", 0.8);
    nh_->setParam("motion_primitive/K_T", 2.2);
    nh_->setParam("motion_primitive/altitude_weight", 0.15);
    nh_->setParam("motion_primitive/distance_weight", 0.02);
    nh_->setParam("motion_primitive/heading_weight", 3.0);
    nh_->setParam("static_obstacles/num", 0);
    nh_->setParam("dynamic_obstacles/num", 0);
    nh_->setParam("waypoints/num", 1);
    nh_->setParam("waypoints/wp1/x", 100.0);
    nh_->setParam("waypoints/wp1/y", 0.0);
    nh_->setParam("waypoints/wp1/h", 20.0);
  }

  void TearDown() override {
    nh_.reset();
    ros::param::del(ns_);
  }

  std::unique_ptr<MotionPrimitives> planner() {
    return std::unique_ptr<MotionPrimitives>(new MotionPrimitives(*nh_));
  }

  MotionPrimitives::s_t state(double altitude = 20.0) const {
    MotionPrimitives::s_t result{};
    result.V = 15.5;
    result.h = altitude;
    result.T = 0.93;
    result.attack = -0.20;
    // Plugin convention: body pitch = -flight_path_angle - attack.
    result.q[0] = std::cos(0.10);
    result.q[2] = std::sin(0.10);
    return result;
  }

  void cylinder(double x, double y, double z, double radius, double height) {
    nh_->setParam("static_obstacles/num", 1);
    nh_->setParam("static_obstacles/obs1/x", x);
    nh_->setParam("static_obstacles/obs1/y", y);
    nh_->setParam("static_obstacles/obs1/z", z);
    nh_->setParam("static_obstacles/obs1/r", radius);
    nh_->setParam("static_obstacles/obs1/height", height);
  }

  void sphere(double x, double y, double z, double radius) {
    nh_->setParam("sphere_obstacles/num", 1);
    nh_->setParam("sphere_obstacles/sphere1/x", x);
    nh_->setParam("sphere_obstacles/sphere1/y", y);
    nh_->setParam("sphere_obstacles/sphere1/z", z);
    nh_->setParam("sphere_obstacles/sphere1/r", radius);
  }

  void horizontalCylinder(double x, double y, double z, double radius,
                          double length, double yaw_degrees) {
    nh_->setParam("horizontal_obstacles/num", 1);
    nh_->setParam("horizontal_obstacles/cylinder1/x", x);
    nh_->setParam("horizontal_obstacles/cylinder1/y", y);
    nh_->setParam("horizontal_obstacles/cylinder1/z", z);
    nh_->setParam("horizontal_obstacles/cylinder1/r", radius);
    nh_->setParam("horizontal_obstacles/cylinder1/length", length);
    nh_->setParam("horizontal_obstacles/cylinder1/axis_yaw", yaw_degrees);
  }

  void expectVerticalEscape(double altitude, double obstacle_z,
                            double obstacle_height, bool climb,
                            double obstacle_x = 30.0) {
    // Eliminate lateral avoidance so a successful choice must use height.
    nh_->setParam("motion_primitive/num", 1);
    nh_->setParam("motion_primitive/theata", 0.0);
    nh_->setParam("waypoints/wp1/h", altitude);
    cylinder(obstacle_x, 0.0, obstacle_z, 3.0, obstacle_height);
    auto mp = planner();
    const auto initial = state(altitude);
    const auto library = mp->generate_library(initial);
    ASSERT_EQ(7u, library.size());

    const auto level = std::min_element(
        library.begin(), library.end(),
        [&](const MotionPrimitives::pr_t &a, const MotionPrimitives::pr_t &b) {
          return std::abs(a.wp.h - altitude) < std::abs(b.wp.h - altitude);
        });
    ASSERT_NE(library.end(), level);
    EXPECT_NEAR(altitude, level->wp.h, 1e-4);
    const auto level_rollout = mp->rollout_motion_primitive(initial, *level);
    EXPECT_TRUE(level_rollout.collision)
        << "Fixture must block the horizontal candidate";

    bool found = false;
    double best_cost = std::numeric_limits<double>::infinity();
    double selected_height = altitude;
    double selected_x = 0;
    std::ostringstream diagnostics;
    for (const auto &primitive : library) {
      const auto trajectory = mp->rollout_motion_primitive(initial, primitive);
      diagnostics << "target=" << primitive.wp.h
                  << " endpoint_height=" << trajectory.states.back().h
                  << " valid=" << trajectory.valid
                  << " collision=" << trajectory.collision << '\n';
      if (!trajectory.valid || trajectory.collision || trajectory.states.empty())
        continue;
      const double cost = mp->evaluate_motion_primitive(initial, primitive);
      if (std::isfinite(cost) && cost < best_cost) {
        found = true;
        best_cost = cost;
        selected_height = trajectory.states.back().h;
        selected_x = trajectory.states.back().pos[0];
      }
    }
    ASSERT_TRUE(found) << "A collision-free vertical escape should exist:\n"
                       << diagnostics.str();
    EXPECT_GT(selected_x, obstacle_x + 3.0)
        << "The selected trajectory must pass the entire obstacle";
    if (climb)
      EXPECT_GT(selected_height, altitude + 1.0);
    else
      EXPECT_LT(selected_height, altitude - 1.0);
  }

  std::string ns_;
  std::unique_ptr<ros::NodeHandle> nh_;
};

TEST_F(MotionPrimitives3D, LibrarySpansThreeDimensions) {
  auto mp = planner();
  const auto initial = state();
  const auto library = mp->generate_library(initial);
  ASSERT_EQ(29u * 7u, library.size());

  std::vector<Point3> endpoints;
  float min_target_height = std::numeric_limits<float>::infinity();
  float max_target_height = -std::numeric_limits<float>::infinity();
  for (const auto &primitive : library) {
    EXPECT_TRUE(std::isfinite(primitive.w_roll_c));
    EXPECT_TRUE(std::isfinite(primitive.attack_c));
    EXPECT_TRUE(std::isfinite(primitive.T_c));
    EXPECT_LE(std::abs(primitive.w_roll_c), 35.0 * M_PI / 180.0 + 1e-5);
    EXPECT_GE(primitive.attack_c, -20.0 * M_PI / 180.0 - 1e-5);
    EXPECT_LE(primitive.attack_c, 25.0 * M_PI / 180.0 + 1e-5);
    EXPECT_GE(primitive.T_c, 0.0);
    EXPECT_LE(primitive.T_c, 20.0 + 1e-5);
    min_target_height = std::min(min_target_height, primitive.wp.h);
    max_target_height = std::max(max_target_height, primitive.wp.h);
    const auto trajectory = mp->rollout_motion_primitive(initial, primitive);
    if (trajectory.valid && !trajectory.collision && !trajectory.states.empty()) {
      const auto &last = trajectory.states.back();
      endpoints.push_back({last.pos[0], last.pos[1], last.h});
    }
  }
  EXPECT_LT(min_target_height, initial.h - 1.0);
  EXPECT_GT(max_target_height, initial.h + 1.0);
  ASSERT_GE(endpoints.size(), 29u);

  const auto altitude_bounds = std::minmax_element(
      endpoints.begin(), endpoints.end(),
      [](const Point3 &a, const Point3 &b) { return a.z < b.z; });
  EXPECT_GT(altitude_bounds.second->z - altitude_bounds.first->z, 2.0);

  // A non-zero scalar triple product rules out one common trajectory plane.
  const Point3 origin = endpoints.front();
  Point3 first{0, 0, 0};
  for (const auto &point : endpoints) {
    const Point3 candidate = difference(point, origin);
    if (dot(candidate, candidate) > dot(first, first)) first = candidate;
  }
  Point3 normal{0, 0, 0};
  for (const auto &point : endpoints) {
    const Point3 candidate = cross(first, difference(point, origin));
    if (dot(candidate, candidate) > dot(normal, normal)) normal = candidate;
  }
  double volume = 0;
  for (const auto &point : endpoints)
    volume = std::max(volume, std::abs(dot(normal, difference(point, origin))));
  EXPECT_GT(volume, 1.0) << "Candidate endpoints are still coplanar";
}

TEST_F(MotionPrimitives3D, FiniteCylinderAllowsClearanceAboveAndBelow) {
  cylinder(25.0, -10.0, 20.0, 5.0, 10.0);
  auto mp = planner();
  EXPECT_TRUE(mp->collision_check(25.0, -10.0, 20.0));
  EXPECT_FALSE(mp->collision_check(25.0, -10.0, 5.0));
  EXPECT_FALSE(mp->collision_check(25.0, -10.0, 35.0));
  EXPECT_FALSE(mp->collision_check(50.0, -10.0, 20.0));
}

TEST_F(MotionPrimitives3D, DynamicCollisionUsesFuturePositionAndHeight) {
  nh_->setParam("dynamic_obstacles/num", 1);
  nh_->setParam("dynamic_obstacles/dyn_obs1/x", 0.0);
  nh_->setParam("dynamic_obstacles/dyn_obs1/y", 0.0);
  nh_->setParam("dynamic_obstacles/dyn_obs1/z", 5.0);
  nh_->setParam("dynamic_obstacles/dyn_obs1/height", 10.0);
  nh_->setParam("dynamic_obstacles/dyn_obs1/r", 2.0);
  nh_->setParam("dynamic_obstacles/dyn_obs1/type", "linear");
  nh_->setParam("dynamic_obstacles/dyn_obs1/velocity", 4.0);
  nh_->setParam("dynamic_obstacles/dyn_obs1/direction", 0.0);
  auto mp = planner();
  EXPECT_TRUE(mp->collision_check(0.0, 0.0, 5.0, 0.0));
  EXPECT_FALSE(mp->collision_check(20.0, 0.0, 5.0, 0.0));
  EXPECT_TRUE(mp->collision_check(20.0, 0.0, 5.0, 5.0));
  EXPECT_FALSE(mp->collision_check(0.0, 0.0, 5.0, 5.0));
  EXPECT_FALSE(mp->collision_check(20.0, 0.0, 20.0, 5.0));
}

TEST_F(MotionPrimitives3D, SphereUsesTheSameClearanceInAllThreeAxes) {
  sphere(25.0, -10.0, 20.0, 5.0);
  auto mp = planner();
  EXPECT_TRUE(mp->collision_check(25.0, -10.0, 20.0));
  // A 5 m sphere has a 1 m isotropic safety margin, including its underside.
  for (const Point3 direction : std::vector<Point3>{
           {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
           {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}) {
    EXPECT_TRUE(mp->collision_check(25 + 5.9 * direction.x,
                                    -10 + 5.9 * direction.y,
                                    20 + 5.9 * direction.z));
    EXPECT_FALSE(mp->collision_check(25 + 6.1 * direction.x,
                                     -10 + 6.1 * direction.y,
                                     20 + 6.1 * direction.z));
  }
}

TEST_F(MotionPrimitives3D, SphereDoesNotOccupyItsBoundingCylinderCorners) {
  sphere(25.0, -10.0, 20.0, 5.0);
  auto mp = planner();
  // Both points are inside the enclosing cylinder, but only one is within
  // the spherical safety envelope. This catches accidental cylinder reuse.
  EXPECT_FALSE(mp->collision_check(25 + 4.8, -10.0, 20 + 4.8));
  EXPECT_TRUE(mp->collision_check(25 + 4.0, -10.0, 20 + 4.0));
  EXPECT_FALSE(mp->collision_check(25.0, -10 + 4.8, 20 - 4.8));
}

TEST_F(MotionPrimitives3D, SphereClearanceIncludesRadiusInflationAndConfiguredMargin) {
  sphere(25.0, 0.0, 30.0, 10.0);
  {
    auto mp = planner();
    // For this larger sphere the 20% radius inflation exceeds the default 1 m.
    EXPECT_TRUE(mp->collision_check(25.0, 0.0, 41.9));
    EXPECT_FALSE(mp->collision_check(25.0, 0.0, 42.1));
  }
  nh_->setParam("motion_primitive/vertical_clearance", 3.0);
  auto mp = planner();
  EXPECT_TRUE(mp->collision_check(37.9, 0.0, 30.0));
  EXPECT_FALSE(mp->collision_check(38.1, 0.0, 30.0));
}

TEST_F(MotionPrimitives3D, SphereMarkerMatchesGeometryAndPreservesCylinderMarkers) {
  cylinder(60.0, 10.0, 5.0, 3.0, 10.0);
  nh_->setParam("dynamic_obstacles/num", 1);
  nh_->setParam("dynamic_obstacles/dyn_obs1/x", 80.0);
  nh_->setParam("dynamic_obstacles/dyn_obs1/y", -10.0);
  visualization_msgs::MarkerArray original;
  {
    auto mp = planner();
    original = mp->collision_visual();
    ASSERT_EQ(2u, original.markers.size());
  }
  sphere(25.0, -10.0, 20.0, 5.0);
  auto mp = planner();
  const auto markers = mp->collision_visual();
  ASSERT_EQ(3u, markers.markers.size());
  for (size_t i = 0; i < original.markers.size(); ++i) {
    EXPECT_EQ(original.markers[i].ns, markers.markers[i].ns);
    EXPECT_EQ(original.markers[i].id, markers.markers[i].id);
    EXPECT_EQ(visualization_msgs::Marker::CYLINDER, markers.markers[i].type);
  }
  const auto &marker = markers.markers.back();
  EXPECT_EQ("sphere_obs1", marker.ns);
  EXPECT_EQ(visualization_msgs::Marker::SPHERE, marker.type);
  EXPECT_EQ("world", marker.header.frame_id);
  EXPECT_DOUBLE_EQ(25.0, marker.pose.position.x);
  EXPECT_DOUBLE_EQ(-10.0, marker.pose.position.y);
  EXPECT_DOUBLE_EQ(20.0, marker.pose.position.z);
  EXPECT_DOUBLE_EQ(1.0, marker.pose.orientation.w);
  EXPECT_DOUBLE_EQ(10.0, marker.scale.x);
  EXPECT_DOUBLE_EQ(10.0, marker.scale.y);
  EXPECT_DOUBLE_EQ(10.0, marker.scale.z);
  EXPECT_FLOAT_EQ(0.0, marker.color.r);
  EXPECT_FLOAT_EQ(0.0, marker.color.g);
  EXPECT_FLOAT_EQ(1.0, marker.color.b);
  EXPECT_FLOAT_EQ(0.5, marker.color.a);
}

TEST_F(MotionPrimitives3D, HorizontalCylinderHasRoundCrossSectionAndFlatEnds) {
  horizontalCylinder(25.0, -10.0, 20.0, 3.0, 40.0, 0.0);
  auto mp = planner();
  // The 1 m margin yields radius 4 m and half-length 21 m.
  EXPECT_TRUE(mp->collision_check(25.0, -10.0, 20.0));
  EXPECT_TRUE(mp->collision_check(45.9, -10.0, 20.0));
  EXPECT_TRUE(mp->collision_check(4.1, -10.0, 20.0));
  EXPECT_FALSE(mp->collision_check(46.1, -10.0, 20.0));
  EXPECT_FALSE(mp->collision_check(3.9, -10.0, 20.0));
  // A capsule would incorrectly extend the occupied volume beyond these caps.
  EXPECT_FALSE(mp->collision_check(46.2, -10.0, 20.0));
  EXPECT_TRUE(mp->collision_check(25.0, -6.1, 20.0));
  EXPECT_FALSE(mp->collision_check(25.0, -5.9, 20.0));
  EXPECT_TRUE(mp->collision_check(25.0, -10.0, 23.9));
  EXPECT_FALSE(mp->collision_check(25.0, -10.0, 24.1));
  EXPECT_TRUE(mp->collision_check(25.0, -10.0, 16.1));
  EXPECT_FALSE(mp->collision_check(25.0, -10.0, 15.9));
  EXPECT_TRUE(mp->collision_check(25.0, -8.0, 23.0));
  EXPECT_FALSE(mp->collision_check(25.0, -7.0, 23.0));
}

TEST_F(MotionPrimitives3D, HorizontalCylinderYawRotatesItsLongAxis) {
  horizontalCylinder(25.0, -10.0, 20.0, 3.0, 40.0, 90.0);
  auto mp = planner();
  EXPECT_TRUE(mp->collision_check(25.0, 10.9, 20.0));
  EXPECT_FALSE(mp->collision_check(25.0, 11.1, 20.0));
  EXPECT_TRUE(mp->collision_check(28.9, -10.0, 20.0));
  EXPECT_FALSE(mp->collision_check(29.1, -10.0, 20.0));
  // This point is along the old x axis, but outside the rotated radial extent.
  EXPECT_FALSE(mp->collision_check(40.0, -10.0, 20.0));
}

TEST_F(MotionPrimitives3D, DiagonalHorizontalCylinderUsesLocalCoordinates) {
  horizontalCylinder(25.0, -10.0, 20.0, 2.0, 20.0, 45.0);
  auto mp = planner();
  const double diagonal = std::sqrt(0.5);
  const auto occupied = [&](double along, double across, double height) {
    return mp->collision_check(25 + diagonal * (along - across),
                               -10 + diagonal * (along + across),
                               20 + height);
  };
  EXPECT_TRUE(occupied(10.9, 0.0, 0.0));
  EXPECT_FALSE(occupied(11.1, 0.0, 0.0));
  EXPECT_TRUE(occupied(5.0, 2.9, 0.0));
  EXPECT_FALSE(occupied(5.0, 3.1, 0.0));
  EXPECT_TRUE(occupied(5.0, 2.0, 2.0));
  EXPECT_FALSE(occupied(5.0, 2.2, 2.2));
}

TEST_F(MotionPrimitives3D, HorizontalMarkerMatchesAxisAndStaticColor) {
  cylinder(60.0, 10.0, 5.0, 3.0, 10.0);
  nh_->setParam("dynamic_obstacles/num", 1);
  nh_->setParam("dynamic_obstacles/dyn_obs1/x", 80.0);
  nh_->setParam("dynamic_obstacles/dyn_obs1/y", -10.0);
  sphere(100.0, 15.0, 20.0, 5.0);
  visualization_msgs::MarkerArray original;
  {
    auto mp = planner();
    original = mp->collision_visual();
    ASSERT_EQ(3u, original.markers.size());
  }
  horizontalCylinder(25.0, -10.0, 20.0, 3.0, 120.0, 37.0);
  auto mp = planner();
  const auto markers = mp->collision_visual();
  ASSERT_EQ(4u, markers.markers.size());
  for (size_t i = 0; i < original.markers.size(); ++i) {
    EXPECT_EQ(original.markers[i].ns, markers.markers[i].ns);
    EXPECT_EQ(original.markers[i].id, markers.markers[i].id);
    EXPECT_EQ(original.markers[i].type, markers.markers[i].type);
  }
  const auto &marker = markers.markers.back();
  EXPECT_EQ("horizontal_obs1", marker.ns);
  EXPECT_EQ(visualization_msgs::Marker::CYLINDER, marker.type);
  EXPECT_EQ("world", marker.header.frame_id);
  EXPECT_DOUBLE_EQ(25.0, marker.pose.position.x);
  EXPECT_DOUBLE_EQ(-10.0, marker.pose.position.y);
  EXPECT_DOUBLE_EQ(20.0, marker.pose.position.z);
  EXPECT_DOUBLE_EQ(6.0, marker.scale.x);
  EXPECT_DOUBLE_EQ(6.0, marker.scale.y);
  EXPECT_DOUBLE_EQ(120.0, marker.scale.z);
  const auto &q = marker.pose.orientation;
  const tf::Vector3 axis = tf::Matrix3x3(tf::Quaternion(q.x, q.y, q.z, q.w)) *
                          tf::Vector3(0, 0, 1);
  EXPECT_NEAR(std::cos(37.0 * M_PI / 180.0), axis.x(), 1e-5);
  EXPECT_NEAR(std::sin(37.0 * M_PI / 180.0), axis.y(), 1e-5);
  EXPECT_NEAR(0.0, axis.z(), 1e-5);
  for (size_t i : std::vector<size_t>{0, 2, 3}) {
    EXPECT_FLOAT_EQ(0.0, markers.markers[i].color.r);
    EXPECT_FLOAT_EQ(0.0, markers.markers[i].color.g);
    EXPECT_FLOAT_EQ(1.0, markers.markers[i].color.b);
    EXPECT_FLOAT_EQ(0.5, markers.markers[i].color.a);
  }
  EXPECT_FLOAT_EQ(1.0, markers.markers[1].color.r);
  EXPECT_FLOAT_EQ(0.0, markers.markers[1].color.g);
  EXPECT_FLOAT_EQ(0.0, markers.markers[1].color.b);
}

TEST_F(MotionPrimitives3D, RolloutDetectsHorizontalRodBetweenClearEndpoints) {
  nh_->setParam("motion_primitive/num", 1);
  nh_->setParam("motion_primitive/vertical_num", 1);
  horizontalCylinder(25.0, 0.0, 20.0, 3.0, 120.0, 90.0);
  auto mp = planner();
  const auto initial = state();
  const auto library = mp->generate_library(initial);
  ASSERT_EQ(1u, library.size());
  const auto trajectory = mp->rollout_motion_primitive(initial, library.front());
  ASSERT_TRUE(trajectory.valid);
  ASSERT_GT(trajectory.states.size(), 2u);
  const auto &end = trajectory.states.back();
  ASSERT_GT(end.pos[0], 30.0);
  EXPECT_FALSE(mp->collision_check(initial.pos[0], initial.pos[1], initial.h));
  EXPECT_FALSE(mp->collision_check(end.pos[0], end.pos[1], end.h));
  EXPECT_TRUE(trajectory.collision);
  EXPECT_GE(mp->evaluate_motion_primitive(initial, library.front()), 10000.0);
}

TEST_F(MotionPrimitives3D, RolloutDetectsSphereBetweenClearEndpoints) {
  nh_->setParam("motion_primitive/num", 1);
  nh_->setParam("motion_primitive/vertical_num", 1);
  sphere(25.0, 0.0, 20.0, 3.0);
  auto mp = planner();
  const auto initial = state();
  const auto library = mp->generate_library(initial);
  ASSERT_EQ(1u, library.size());
  const auto trajectory = mp->rollout_motion_primitive(initial, library.front());
  ASSERT_TRUE(trajectory.valid);
  ASSERT_GT(trajectory.states.size(), 2u);
  const auto &end = trajectory.states.back();
  ASSERT_GT(end.pos[0], 30.0);
  EXPECT_FALSE(mp->collision_check(initial.pos[0], initial.pos[1], initial.h));
  EXPECT_FALSE(mp->collision_check(end.pos[0], end.pos[1], end.h));
  EXPECT_TRUE(trajectory.collision);
  EXPECT_GE(mp->evaluate_motion_primitive(initial, library.front()), 10000.0);
}

TEST_F(MotionPrimitives3D, GroundCrossingRejectsEvenTheClimbCandidate) {
  auto mp = planner();
  auto initial = state(1.1);
  initial.gamma = -0.35;
  const auto library = mp->generate_library(initial);
  ASSERT_FALSE(library.empty());
  const auto highest = std::max_element(
      library.begin(), library.end(),
      [](const MotionPrimitives::pr_t &a, const MotionPrimitives::pr_t &b) {
        return a.wp.h < b.wp.h;
      });
  const auto trajectory = mp->rollout_motion_primitive(initial, *highest);
  EXPECT_TRUE(trajectory.collision);
}

TEST_F(MotionPrimitives3D, CeilingCrossingRejectsEvenTheDescentCandidate) {
  auto mp = planner();
  auto initial = state(79.9);
  initial.gamma = 0.35;
  const auto library = mp->generate_library(initial);
  ASSERT_FALSE(library.empty());
  const auto lowest = std::min_element(
      library.begin(), library.end(),
      [](const MotionPrimitives::pr_t &a, const MotionPrimitives::pr_t &b) {
        return a.wp.h < b.wp.h;
      });
  const auto trajectory = mp->rollout_motion_primitive(initial, *lowest);
  EXPECT_TRUE(trajectory.collision);
}

TEST_F(MotionPrimitives3D, DelayCompensationPreservesSelectedHeight) {
  nh_->setParam("motion_primitive/num", 1);
  nh_->setParam("motion_primitive/theata", 0.0);
  auto mp = planner();
  const auto initial = state();
  const auto library = mp->generate_library(initial);
  ASSERT_FALSE(library.empty());
  const auto climb = std::find_if(
      library.begin(), library.end(),
      [&](const MotionPrimitives::pr_t &primitive) {
        return primitive.wp.h > initial.h + 1.0;
      });
  ASSERT_NE(library.end(), climb);
  ASSERT_GT(climb->wp.h, initial.h + 1.0);
  const auto level = std::min_element(
      library.begin(), library.end(),
      [&](const MotionPrimitives::pr_t &a, const MotionPrimitives::pr_t &b) {
        return std::abs(a.wp.h - initial.h) < std::abs(b.wp.h - initial.h);
      });
  ASSERT_NE(library.end(), level);
  const auto compensated = mp->compensate_cmd(*climb, initial);
  const auto level_compensated = mp->compensate_cmd(*level, initial);
  EXPECT_FLOAT_EQ(climb->wp.h, compensated.wp.h);
  EXPECT_TRUE(std::isfinite(compensated.attack_c));
  EXPECT_TRUE(std::isfinite(compensated.T_c));
  EXPECT_GT(compensated.attack_c, level_compensated.attack_c + 0.01);
  const auto climb_path = mp->rollout_motion_primitive(initial, compensated);
  const auto level_path = mp->rollout_motion_primitive(initial, level_compensated);
  ASSERT_TRUE(climb_path.valid);
  ASSERT_TRUE(level_path.valid);
  ASSERT_FALSE(climb_path.states.empty());
  ASSERT_FALSE(level_path.states.empty());
  EXPECT_GT(climb_path.states.back().h, level_path.states.back().h + 1.0);
}

TEST_F(MotionPrimitives3D, VisualizedTrajectoryMatchesEvaluatedRollout) {
  auto mp = planner();
  const auto initial = state();
  const auto library = mp->generate_library(initial);
  ASSERT_FALSE(library.empty());
  const auto climb = std::find_if(
      library.begin(), library.end(),
      [&](const MotionPrimitives::pr_t &primitive) {
        return primitive.wp.h > initial.h + 1.0;
      });
  ASSERT_NE(library.end(), climb);
  const auto trajectory = mp->rollout_motion_primitive(initial, *climb);
  ASSERT_TRUE(trajectory.valid);
  const auto marker = mp->visual_motion_primitive(initial, *climb);
  ASSERT_EQ(trajectory.states.size(), marker.points.size());
  ASSERT_GT(marker.points.size(), 1u);
  for (size_t i = 0; i < marker.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(trajectory.states[i].pos[0], marker.points[i].x);
    EXPECT_DOUBLE_EQ(trajectory.states[i].pos[1], marker.points[i].y);
    EXPECT_DOUBLE_EQ(trajectory.states[i].h, marker.points[i].z);
  }
  EXPECT_GT(marker.points.back().z, marker.points.front().z + 1.0);
}

TEST_F(MotionPrimitives3D, ChoosesClimbWhenHorizontalPassageIsBlocked) {
  expectVerticalEscape(5.0, 3.0, 6.0, true);
}

TEST_F(MotionPrimitives3D, ChoosesDescentBelowAnElevatedObstacle) {
  // Descending needs additional lead distance because attack has a 1 s lag.
  // The cylinder's lower face is at 19 m, with another 1 m of clearance.
  expectVerticalEscape(20.0, 35.0, 32.0, false, 35.0);
}

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "motion_primitives_3d_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
