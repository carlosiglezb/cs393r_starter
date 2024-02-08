//
// Created by Carlos on 2/4/24.
//

#include "car_obstacle_avoidance.h"
#include <iostream>

namespace nav_utils {

CarObstacleAvoidance::CarObstacleAvoidance(float car_width,
                                           float car_length,
                                           float car_wheelbase,
                                           unsigned int n_paths,
                                           unsigned int n_scan_points,
                                           float car_height) :
  free_paths_pc_vec_(n_scan_points),
  candidate_curvatures_(n_paths) {

  // Initialize main modules
  obstacle_detection_ = std::make_shared<CarObstacleDetection>(car_width, car_length, car_wheelbase, car_height);
  score_mgr_ = std::make_shared<PathScoreManager>(n_paths);
  toc_controller_ = std::make_shared<CurvedPath>(0.);

  // Initialize path parameters (min/max curvature)
  c_min_ = -1.;
  c_max_ = 1.;

  // Initialize values computed in intermediate calculations
  curve_collision_free_fp_ = 0.;

  // Initialize control outputs
  cmd_vel_ = 0.;
  cmd_curvature_ = 0.;

  // Initialize world frame to coincide with car's initial pose
  w_p_car_.setZero();
  theta_ = 0.;

  // Create vector of curvatures to sample / evaluate
  float curvature_inc = (c_max_ - c_min_) / float(n_paths - 1);
  for (unsigned int i = 0; i < n_paths; i++) {
    candidate_curvatures_[i] = (c_max_ - i * curvature_inc);
  }
}

CarObstacleAvoidance::~CarObstacleAvoidance() = default;

void CarObstacleAvoidance::doControl(const std::vector<Eigen::Vector2f> &point_cloud,
                                     const Eigen::Vector2f &w_p_goal) {
  float dist_to_goal, r;
  float clearance;
  unsigned int min_free_path_idx, max_score_idx;
  float dt = getControllerDt();
  Eigen::Vector2f w_p_car_end_fp;

  // Start clean
  score_mgr_->reset();
  for (auto & curvature : candidate_curvatures_) {
    clearance = 100.;
    resetFreePathsVec();
    // Update current curvature
    r = float(1.) / curvature;
    obstacle_detection_->setCurrentCurvature(curvature);

    // Evaluate score from each point in scan
    for (unsigned int p_i = 0; p_i < point_cloud.size(); p_i++) {
      free_paths_pc_vec_[p_i] = obstacle_detection_->estimateFreePath(point_cloud[p_i]);

      // chop free path to radial distance
      float path_poa = obstacle_detection_->freePathToClosestPoA(w_p_goal, w_p_car_, theta_);
      free_paths_pc_vec_[p_i] = std::min(free_paths_pc_vec_[p_i], path_poa);  // TODO check if this should be moved

      // Discard point if in collision path and is before reaching the goal
      if (obstacle_detection_->isInCollision() && (point_cloud[p_i].norm() < w_p_goal.norm())) {
        // exit loop as soon as one point is in collision
        if (obstacle_detection_->isInCollision() && (free_paths_pc_vec_[p_i] < path_poa)) {
          free_paths_pc_vec_[p_i] = -10000.;
          clearance = 0.;
          break;
        }
      }

      // compute clearance
      clearance = std::min(clearance, obstacle_detection_->computeClearance(point_cloud[p_i]));
    }

    // the shortest path corresponds to the point that it will collide with first
    min_free_path_idx = std::min_element(
            free_paths_pc_vec_.begin(), free_paths_pc_vec_.end()) - free_paths_pc_vec_.begin();
    curve_collision_free_fp_ = free_paths_pc_vec_[min_free_path_idx];

    // update estimated position of car at end of free path
    if (curve_collision_free_fp_ > 0.) {
      if (std::abs(curvature) < 0.01) {  // assuming straight line
        w_p_car_end_fp.x() = w_p_car_.x() + curve_collision_free_fp_ * std::cos(theta_);
        w_p_car_end_fp.y() = w_p_car_.y() + curve_collision_free_fp_ * std::sin(theta_);
        dist_to_goal = (w_p_goal - w_p_car_end_fp).norm();
      } else {
        float phi_min = free_paths_pc_vec_[min_free_path_idx] / r;
        w_p_car_end_fp.x() = w_p_car_.x() + r * std::sin(phi_min);
        w_p_car_end_fp.y() = w_p_car_.y() + r * (1. - std::cos(phi_min));
        dist_to_goal = (w_p_goal - w_p_car_end_fp).norm();
      }
    } else {  // if current path leads to a collision, use current distance to goal
//      curve_collision_free_fp_ = 0.;
      dist_to_goal = (w_p_goal - w_p_car_).norm();
    }

    // Update score manager
    score_mgr_->computeAndStore(curve_collision_free_fp_, clearance, dist_to_goal);
  }

  // Choose path with max score and apply control
  max_score_idx = score_mgr_->getMaximumScoreIdx();

  // Update control
  cmd_vel_ = toc_controller_->compute1DTOC(score_mgr_->getFpDistance(max_score_idx));
  cmd_curvature_ = candidate_curvatures_[max_score_idx];

  // Update state estimate for next time step
  w_p_car_.x() += cmd_vel_ * std::cos(cmd_curvature_) * dt;
  w_p_car_.y() += cmd_vel_ * std::sin(cmd_curvature_) * dt;
  theta_ += cmd_vel_ * cmd_curvature_ * dt;
}

void CarObstacleAvoidance::resetFreePathsVec() {
  for (auto & path : free_paths_pc_vec_) {
    path = 0.;
  }
}

float CarObstacleAvoidance::getCmdVel() const {
  return cmd_vel_;
}

float CarObstacleAvoidance::getCmdCurvature() const {
  return cmd_curvature_;
}

float CarObstacleAvoidance::getControllerDt() const {
  return toc_controller_->getDt();
}

Eigen::Vector2f CarObstacleAvoidance::getPosEst() const {
  return w_p_car_;
}

} // nav_utils