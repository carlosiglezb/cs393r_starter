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
using std::max;
using std::min;
using std::tuple;
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

  // Global planner
  w_next_waypoint_(0., 0.);     // goal in world frame
  robot_pos_ = Point(0., 0.);
  robot_goal_ = Point(0., 0.);
  minDistanceToObstacle_ = 0.2;
//  rrt_ = RRT(1.);
//  rrt_.readObstaclesFromFile(GetMapFileFromName(map_name));
  rrt_star_ = RRTStar(1., 1.);
  rrt_star_.readObstaclesFromFile(GetMapFileFromName(map_name));
  n_waypoint_count_ = 1;
  b_nav_goal_set_ = false;
  b_last_waypoint_ = false;
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
//  rrt_.generate(robot_pos_, robot_goal_);
  rrt_star_.generate(robot_pos_, robot_goal_);

// // Set first waypoint
//   n_waypoint_count_++;
// //  w_next_waypoint_ << rrt_.getRRTPathPoint(n_waypoint_count_);
//   w_next_waypoint_ << rrt_star_.getRRTPathPoint(n_waypoint_count_);
//   b_nav_goal_set_ = true;
//   nav_complete_ = false;

  // Reverse the order of waypoints
  n_waypoint_count_ = rrt_star_.getRRTPathPoints().size() - 1;
  w_next_waypoint_ << rrt_star_.getRRTPathPoint(n_waypoint_count_);
  b_nav_goal_set_ = true;
  nav_complete_ = false;
  b_last_waypoint_ = (n_waypoint_count_ == 0);
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
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

  // Drive commands:
  float new_cur, new_distance;
  std::tie(new_cur, new_distance) = GetCurvature();
  float new_vel = GetVelocity(new_distance);
  drive_msg_.curvature = new_cur;
  drive_msg_.velocity = new_vel;

   // Move waypoint away by two more meters in x-direction
   if (!b_last_waypoint_ && ((w_next_waypoint_ - robot_loc_).norm() - 1.0 < 0.)) {
    //  n_waypoint_count_++;
     n_waypoint_count_--; // Reverse the order of waypoints
     std::cout << "Waypoint Index: " << n_waypoint_count_ << std::endl;
//     w_next_waypoint_ = rrt_.getRRTPathPoint(n_waypoint_count_);
     w_next_waypoint_ = rrt_star_.getRRTPathPoint(n_waypoint_count_);
    //  b_last_waypoint_ = n_waypoint_count_ == int(rrt_star_.getRRTPathPoints().size());
     b_last_waypoint_ = (n_waypoint_count_ == 0); // Reverse the order of waypoints
   } else {   // if last waypoint, just check if we are close enough to the goal
     if ((w_next_waypoint_ - robot_loc_).norm() < 0.1) {
       nav_complete_ = true;
     }
   }

  // Reverse the order of waypoints
  // if (!b_last_waypoint_ && ((w_next_waypoint_ - robot_loc_).norm() - 1.0 < 0.)) {
  //   n_waypoint_count_--;
  //   std::cout << "Waypoint Index: " << n_waypoint_count_ << std::endl;
  //   w_next_waypoint_ = rrt_star_.getRRTPathPoint(n_waypoint_count_);
  //   b_last_waypoint_ = (n_waypoint_count_ == 0);
  // } else {
  //   if ((w_next_waypoint_ - robot_loc_).norm() < 0.1) {
  //     nav_complete_ = true;
  //   }
  // }

  // Print the index and coordinates of each waypoint
  // const auto& pathPoints = rrt_star_.getRRTPathPoints();
  // for (size_t i = 0; i < pathPoints.size(); ++i) {
  //   const auto& point = pathPoints[i];
  //   visualization::DrawPoint(point, 0x0000FF, global_viz_msg_); 
  //   std::cout << "Waypoint Index: " << i << ", Coordinates: (" << point[0] << ", " << point[1] << ")" << std::endl;
  // }


  // Visualize RRT path waypoints
  for (auto point : rrt_star_.getRRTPathPoints()) {
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

float GetDistance(const Vector2f& point1, const Vector2f& point2) {
  return sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2));
}

tuple<float, float> Navigation::GetCurvature() {
  const float car_length = 0.535;
  const float car_width = 0.281;
  const float wheel_base = 0.324;
  const float max_curvature = 1.0;

  const float safety_margin = 0.1;

  const float base_link_to_side = car_width / 2;
  const float base_link_to_front = (car_length + wheel_base) / 2;

  // Vector2f goal(5, 0);
  float px = w_next_waypoint_[0] - robot_loc_[0];
  float py = w_next_waypoint_[1] - robot_loc_[1];
  // visualization::DrawCross(nav_goals_[nav_goal_idx_], 0.1, 0xFF0000, global_viz_msg_);
  Vector2f goal(px*cos(robot_angle_)+py*sin(robot_angle_), -px*sin(robot_angle_)+py*cos(robot_angle_));
  visualization::DrawCross(goal, 0.5, 0xFF0000, local_viz_msg_);
  // Vector2f goal = w_next_waypoint_;
  // Vector2f goal = nav_goals_[nav_goal_idx_];
  Vector2f base_link(0, 0);

  // for (Vector2f point: point_cloud_) {
  //   visualization::DrawCross(point, 0.01, 0xFF0000, local_viz_msg_);
  // }

  vector<float> curvature_candidates;
  // for (float i = -0.5; i <= 0.5; i += 0.1) {
  //   if (last_curvature_ + i >= -max_curvature && last_curvature_ + i <= max_curvature) {
  //     curvature_candidates.push_back(last_curvature_ + i);
  //   }
  // }
  for (float i = -max_curvature; i <= max_curvature; i += 0.05) {
    curvature_candidates.push_back(i);
  }
  // const vector<float> curvature_candidates {-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0};
  // const vector<float> curvature_candidates {-1.0, -0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
  vector<float> scores;
  vector<float> free_path_lengths;
  for (float curvature_candidate: curvature_candidates) {
    Vector2f point_of_interest;
    float free_path_length, clearance, distance_to_goal;
    if (curvature_candidate > -0.01 && curvature_candidate < 0.01) {
      point_of_interest << goal[0] + base_link_to_front + safety_margin, 0;
      free_path_length = goal[0];
      clearance = 0.5;
      distance_to_goal = goal[1];
      for (Vector2f point: point_cloud_) {
        if (point[0] <= base_link_to_front + safety_margin) {
          continue;
        }
        if (point[0] >= goal[0] + base_link_to_front + safety_margin) {
          continue;
        }
        if (abs(point[1]) < base_link_to_side + safety_margin) {  // hit front
          if (point[0] - base_link_to_front - safety_margin < free_path_length) {
            point_of_interest << point[0], point[1];
            // distance_to_goal = goal[0] - point[0] + safety_margin + base_link_to_front;
            distance_to_goal = GetDistance(point_of_interest, goal);
          }
          free_path_length = min(free_path_length, point[0] - base_link_to_front - safety_margin);
          clearance = 0;
        } else {
          clearance = min(clearance, abs(point[1]) - base_link_to_side - safety_margin);
          // distance_to_goal = min(distance_to_goal, abs(goal[1]));
        }
      }
      // scores.push_back(free_path_length + 3 * clearance - 2.0 * distance_to_goal);
      // free_path_lengths.push_back(free_path_length);

      // visualization::DrawCross(point_of_interest, 0.1, 0xFF0000, local_viz_msg_);
      Vector2f free_path_point(free_path_length, 0);
      // visualization::DrawLine(base_link, base_link + free_path_point, 0xFF0000, local_viz_msg_);
      continue;
    } else {
      float turning_radius = abs(1 / curvature_candidate);
      // float steering_angle = atan(wheel_base / turning_radius);
      float min_radius = turning_radius - base_link_to_side - safety_margin;
      Vector2f turning_center;
      Vector2f front_close;
      Vector2f front_far;
      if (curvature_candidate < 0) {
        turning_center << 0.0, -turning_radius;
        front_close << base_link_to_front + safety_margin, -base_link_to_side - safety_margin;
        front_far << base_link_to_front + safety_margin, base_link_to_side + safety_margin;
      } else {
        turning_center << 0.0, turning_radius;
        front_close << base_link_to_front + safety_margin, base_link_to_side + safety_margin;
        front_far << base_link_to_front + safety_margin, -base_link_to_side - safety_margin;
      }
      float max_radius = GetDistance(turning_center, front_far);
      float front_radius = GetDistance(turning_center, front_close);

      float max_free_path_angle = atan(goal[0] / turning_radius);
      // float max_free_path_angle = atan(1);
      // float angle_of_interest = max_free_path_angle;
      // free_path_length = min(max_free_path_angle * turning_radius, goal[0]);
      free_path_length = 10;
      distance_to_goal = abs(GetDistance(goal, turning_center) - turning_radius);
      clearance = 0.5;
      if (turning_radius > front_radius) {
        max_free_path_angle += asin(base_link_to_front / turning_radius);
      } else {
        max_free_path_angle += acos((turning_radius - base_link_to_side) / turning_radius);
      }
      if (curvature_candidate > 0) {
        point_of_interest << sin(max_free_path_angle) * turning_radius, (1-cos(max_free_path_angle))*turning_radius;
      } else {
        point_of_interest << sin(max_free_path_angle) * turning_radius, (cos(max_free_path_angle)-1)*turning_radius;
      }

      for (Vector2f point: point_cloud_) {
        if (point[0] <= front_far[0]) {
          continue;
        }
        if (abs(point[1]) > abs(turning_center[1])) {
          continue;
        }
        float angle = atan(point[0] / abs(turning_center[1] - point[1]));
        // if (angle >= max_free_path_angle) {
        //   continue;
        // }
        if (curvature_candidate > 0 && point[1] < front_far[1]) {
          continue;
        } else if (curvature_candidate < 0 && point[1] > front_far[1]) {
          continue;
        }
        float distance_to_center = GetDistance(point, turning_center);
        if (distance_to_center < min_radius) {
          clearance = min(clearance, min_radius - distance_to_center);
        } else if (distance_to_center > max_radius) {
          clearance = min(clearance, distance_to_center - max_radius);
        } else {
          float hit_point_angle;
          if (distance_to_center > front_radius) {  // hit front
            hit_point_angle = asin(front_far[0] / distance_to_center);
          } else {  // hit side
            hit_point_angle = acos(min_radius / distance_to_center);
          }
          if (angle > hit_point_angle) {
            if ((angle - hit_point_angle) * turning_radius < free_path_length) {
              // angle_of_interest = angle;
              point_of_interest << point[0], point[1];
              Vector2f free_path_point(turning_radius * sin(hit_point_angle), turning_radius * (1 - cos(hit_point_angle)));
              distance_to_goal = GetDistance(free_path_point, goal);
              clearance = 0;
            }
            free_path_length = min(free_path_length, (angle - hit_point_angle) * turning_radius);

          }
          // else {
          //   free_path_length = 0;
          // }
        }
      }
      // scores.push_back(free_path_length + 3 * clearance - 2.0 * distance_to_goal);
      // free_path_lengths.push_back(free_path_length);

      // visualization::DrawCross(point_of_interest, 0.1, 0xFF0000, local_viz_msg_);
      // if (curvature_candidate > 0) {
      //   visualization::DrawArc(turning_center, turning_radius, -atan(1)*2, angle_of_interest-atan(1)*2, 0xFF0000, local_viz_msg_);
      // } else {
      //   visualization::DrawArc(turning_center, turning_radius, atan(1)*2-angle_of_interest, atan(1)*2, 0xFF0000, local_viz_msg_);
      // }
    }
    scores.push_back(free_path_length + 3 * clearance - 10.0 * distance_to_goal);
    free_path_lengths.push_back(free_path_length);
  }

  float max_score = -1000;
  float curvature = 0;
  float distance_to_goal = 0;
  for (size_t i = 0; i < scores.size(); i++) {
    // cout << std::fixed << std::setprecision(2) << "Cur: " << curvature_candidates[i] << "\t S: "<< scores[i] << "\t FPL: " << free_path_lengths[i] << "\n";
    if (scores[i] > max_score) {
      max_score = scores[i];
      curvature = curvature_candidates[i];
      distance_to_goal = free_path_lengths[i];
    }
  }
  return std::make_tuple(curvature, distance_to_goal);
}

float Navigation::GetVelocity(float distance_to_goal) {  // TOC
  const float max_acc = 4;
  const float max_dec = 4.0;
  const float max_vel = 1;
  const float update_interval = 0.05;
  const float latency = 0.1;

  float x_3 = pow(robot_vel_[0], 2) / (2 * max_dec) + latency * robot_vel_[0];
  x_3 += robot_vel_[0] * latency;
  if (x_3 >= distance_to_goal) {  // Deceleration
    return robot_vel_[0] - max_dec * update_interval;
  } else if (robot_vel_[0] >= max_vel) {  // Cruise
    return max(robot_vel_[0] - max_dec * update_interval, max_vel);
  } else {
    return min(robot_vel_[0] + max_acc * update_interval, max_vel);
  }
  return 0;
}

}  // namespace navigation
