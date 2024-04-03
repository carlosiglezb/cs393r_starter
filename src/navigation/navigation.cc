//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "../visualization/visualization.h"
#include <iostream>

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;
using namespace nav_utils;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);

  // Car dimensions.
  float car_width = 0.281;
  float car_length = 0.535;
  float car_wheelbase = 0.324; // with bumper length might be 0.5
//  float car_height = 0.15;

  // Controller parameters
  n_paths_ = 21;
  unsigned int n_scan_points = 481;   // total are 1081

  // Obstacle Avoidance Controller
  oa_controller_ = std::make_shared<CarObstacleAvoidance>(car_width, car_length,
                                        car_wheelbase, n_paths_, n_scan_points);
  w_next_waypoint_(0., 0.);     // goal in world frame

  // Global planner
  robot_pos_ = Point(0., 0.);
  robot_goal_ = Point(0., 0.);
  rrt_ = RRT(1.);
  rrt_.readObstaclesFromFile(GetMapFileFromName(map_name));
  n_waypoint_count_ = 1;
  b_nav_goal_set_ = false;
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {

  if (!odom_initialized_) {
    std::cout << "Initialize odometry first" << std::endl;
    return;
  }

  // Planning with RRT
  std::cout << "SetPose was at: " << robot_loc_.transpose() << std::endl;
  robot_pos_ = Point(robot_loc_.x(), robot_loc_.y());
  std::cout << "NavGoal set to: " << loc.transpose() << std::endl;
  robot_goal_ = Point(loc.x(), loc.y());
  rrt_.generate(robot_pos_, robot_goal_);

  // Set first waypoint
  n_waypoint_count_++;
  w_next_waypoint_ << rrt_.getRRTPathPoint(n_waypoint_count_);
  b_nav_goal_set_ = true;
  nav_complete_ = false;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
  oa_controller_->setCarPose(loc, angle);
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // If planner has not finished, we can't do anything.
  if (!b_nav_goal_set_) return;

  if (nav_complete_) {
    drive_msg_.velocity = 0.;
    drive_msg_.curvature = 0.;
    drive_pub_.publish(drive_msg_);
    return;
  }

  //
  // Obstacle Avoidance Control loop
  //
//  std::cout << "w_next_waypoint_: " << w_next_waypoint_.transpose() << std::endl;
//  std::cout << "robot_loc_: " << robot_loc_.transpose() << std::endl;
  oa_controller_->doControl(point_cloud_, w_next_waypoint_);

  // Drive commands:
   drive_msg_.curvature = oa_controller_->getCmdCurvature();
   drive_msg_.velocity = oa_controller_->getCmdVel();

   // Move waypoint away by two more meters in x-direction
   if ((w_next_waypoint_ - robot_loc_).norm() - 1.0 < 0.) {
     n_waypoint_count_++;
     std::cout << "Waypoint Index: " << n_waypoint_count_ << std::endl;
     w_next_waypoint_ = rrt_.getRRTPathPoint(n_waypoint_count_);
     if (n_waypoint_count_ == int(rrt_.getRRTPathPoints().size())) {
       nav_complete_ = true;
     }
   }

  // Visualize commanded path
  unsigned int max_score = oa_controller_->getMaxScoreIdx();
  visualization::DrawPathOption(oa_controller_->getCurvature(max_score),
                 oa_controller_->getFpDistance(max_score),
                 oa_controller_->getClearance(max_score),
                 0xFF0000,    // red color
                 true,
                 local_viz_msg_);

  // Visualize RRT path waypoints
  for (auto point : rrt_.getRRTPathPoints()) {
    visualization::DrawPoint(point, 0x0000FF, global_viz_msg_);
  }

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
