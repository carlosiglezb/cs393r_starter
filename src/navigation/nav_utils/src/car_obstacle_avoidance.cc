//
// Created by Carlos on 2/4/24.
//

#include "car_obstacle_avoidance.h"

namespace nav_utils {

CarObstacleAvoidance::CarObstacleAvoidance(float car_width,
                                           float car_length,
                                           float car_wheelbase,
                                           unsigned int n_paths,
                                           unsigned int n_scan_points,
                                           float car_height) :
        free_paths_pc_vec_(n_scan_points), candidate_curvatures_(n_paths) {

  // Initialize main modules
  obstacle_detection_ = std::make_shared<CarObstacleDetection>(car_width, car_length, car_wheelbase, car_height);
  score_mgr_ = std::make_shared<PathScoreManager>(n_paths);
  toc_controller_ = std::make_shared<CurvedPath>(0.);

  // Initialize path parameters
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
  float curvature_inc = (c_max_ - c_min_) / float(n_paths);
  for (unsigned int i = 0; i < n_paths; i++) {
    candidate_curvatures_[i] = (c_max_ - i * curvature_inc);
  }
}

CarObstacleAvoidance::~CarObstacleAvoidance() {}

void CarObstacleAvoidance::doControl(const std::vector<Eigen::Vector2f> &point_cloud,
                                     const Eigen::Vector2f &w_p_goal) {
  float dist_to_goal;
  float clearance = 0.1;
  unsigned int min_free_path_idx, max_score_idx;
  float dt = getControllerDt();

  // Start clean
  score_mgr_->resetIdx();
  for (auto & curvature : candidate_curvatures_) {
    // Update current curvature
    obstacle_detection_->setCurrentCurvature(curvature);

    // Evaluate score from each point in scan
    for (unsigned int p_i = 0; p_i < point_cloud.size(); p_i++) {
      free_paths_pc_vec_[p_i] = obstacle_detection_->estimateFreePath(point_cloud[p_i]);

      // chop free path to radial distance
      float path_poa = obstacle_detection_->freePathToClosestPoA(w_p_goal, w_p_car_, theta_);
      free_paths_pc_vec_[p_i] = std::min(free_paths_pc_vec_[p_i], path_poa);
    }

    // the shortest path corresponds to the point that it will collide with first
    min_free_path_idx = std::min_element(
            free_paths_pc_vec_.begin(), free_paths_pc_vec_.end()) - free_paths_pc_vec_.begin();
    curve_collision_free_fp_ = free_paths_pc_vec_[min_free_path_idx];
    dist_to_goal = (w_p_goal - w_p_car_).norm();

    // Update score manager
    score_mgr_->computeAndStore(curve_collision_free_fp_, clearance, dist_to_goal);
  }

  // Choose path with max score and apply control
  max_score_idx = score_mgr_->getMaximumScoreIdx();

  // Update control
  cmd_vel_ = toc_controller_->compute1DTOC(score_mgr_->getDistanceToGoal(max_score_idx));
  cmd_curvature_ = candidate_curvatures_[max_score_idx];

  // Update state estimate for next time step
  w_p_car_.x() += cmd_vel_ * std::cos(cmd_curvature_) * dt;
  w_p_car_.y() += cmd_vel_ * std::sin(cmd_curvature_) * dt;
  theta_ += cmd_vel_ * cmd_curvature_ * dt;

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


} // nav_utils