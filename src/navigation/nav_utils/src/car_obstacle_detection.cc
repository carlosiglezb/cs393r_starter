//
// Created by Carlos on 2/3/24.
//

#include "car_obstacle_detection.h"
#include <math.h>

namespace nav_utils {

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

CarObstacleDetection::CarObstacleDetection(float car_width,
                                           float car_length,
                                           float car_wheelbase,
                                           float car_height) {
  r_min_ = 0.;
  r_max_ = 0.;
  phi_ = 0.;
  phi_poa_ = 0.;
  c_vec_.setZero();
  current_curvature_ = 0.;
  curvature_zero_thresh_ = 0.01;

  car_width_ = car_width;
  car_length_ = car_length;
  car_wheelbase_ = car_wheelbase;
  car_height_ = car_height;

  min_turn_radius_ = 0.98;
  b_p_in_collision_ = false;
}

CarObstacleDetection::~CarObstacleDetection() {}

void CarObstacleDetection::computeMinMaxCurvatureRadius(float curvature) {
  // Do not compute if curvature is close to zero
  if (abs(curvature) < curvature_zero_thresh_) {
    r_min_ = 0.;
    r_max_ = 0.;
    return;
  }

  float r = float(1.) / curvature;
  int sign = sgn(curvature);
  r_min_ = r - sign * car_width_;
  r_max_ = sign * std::sqrt( (r + sign*car_width_)*(r + sign*car_width_) + car_length_*car_length_ );
  c_vec_.setZero();
  c_vec_.y() = r;
}

float CarObstacleDetection::estimateFreePath(const Eigen::Vector2f &point_cloud_2D) {
  // Get parameters of interest
  float x = point_cloud_2D[0];
  float y = point_cloud_2D[1];

  // if curvature is near zero, assume straight motion
  if (std::abs(current_curvature_) < curvature_zero_thresh_) {
    // Check if point in collision (in front of car and within car width)
    if ((x < 0) || (std::abs(y) > car_width_)) {   // not in collision, skip point
      b_p_in_collision_ = false;
      return point_cloud_2D.norm();   // TODO replace with range of laser?
    }

    // Otherwise, it's in collision
    b_p_in_collision_ = true;
    return x - car_length_;
  }

  // If we reach here, we're not moving with curvature near zero
  float r = float(1.) / current_curvature_;
  float theta, omega;

  // Using trig, we compute the free path length as follows
  if (r > 0){
    theta = std::atan2(x, r - y);   // angle from base to given point cloud
    omega = std::atan2(car_length_, r - car_width_);  // (angle) base -> free path
  } else {
    theta = std::atan2(-x, y - r);   // angle from base to given point cloud
    omega = std::atan2(-car_length_, car_width_ - r);  // (angle) base -> free path
  }

  // Check if point is inside collision path
  float radial_dist_to_point = (point_cloud_2D - c_vec_).norm();
  if ((radial_dist_to_point >= r_min_) && (radial_dist_to_point <=  r_max_) && theta > 0.){
    // if turning left
    b_p_in_collision_ = true;
    phi_ = theta - omega;
    return r * phi_;   // free path length
  } else if ((-radial_dist_to_point <= r_min_) && (-radial_dist_to_point >=  r_max_) && theta < 0.) {
    // if turning right
    b_p_in_collision_ = true;
    phi_ = theta - omega;
    return r * phi_;   // free path length
  } else {  // not in collision path
    b_p_in_collision_ = false;
    return point_cloud_2D.norm();   // TODO replace with range of laser?
  }
}

float CarObstacleDetection::freePathToClosestPoA(const Eigen::Vector2f &w_p_goal,
                                                 const Eigen::Vector2f &w_p_car,
                                                 const float theta) {
  float r = float(1.) / current_curvature_;
  // Get coordinate of center of circle in world coordinates
  Eigen::Vector2f w_p_circ = w_p_car;
  Eigen::Matrix2f w_R_c;
  w_R_c << cos(theta), -sin(theta),
            sin(theta), cos(theta);
  Eigen::Vector2f base_up_vec(0., r);
  Eigen::Vector2f w_up_vec = w_R_c * base_up_vec;
  w_p_circ += w_up_vec;
  
  // Vector from center of circle towards goal of length 'r'
  Eigen::Vector2f vec_circ_to_g = (w_p_goal - w_p_circ);
  vec_circ_to_g = vec_circ_to_g * abs(r) / vec_circ_to_g.norm();

  // Get angle between vectors
  phi_poa_ = acos(float (-w_up_vec.transpose() * vec_circ_to_g) / (r * r));
  return abs(r) * phi_poa_;
}

float CarObstacleDetection::computeClearance(const Eigen::Vector2f &point) {
  float clearance = 0.;
  float dist_to_point = (c_vec_ - point).norm();

  // Compute clearance
  if (dist_to_point > c_vec_.y()) {
    // if point hits front of car
    clearance = dist_to_point - r_max_;
  } else if (dist_to_point < c_vec_.y()) {
    // if point is inside path towards inner radius
    clearance = r_min_ - dist_to_point;
  }

  // if clearance is negative, we will eventually hit the point
  return clearance;
}

bool CarObstacleDetection::isInCollision() const {
  return b_p_in_collision_;
}

void CarObstacleDetection::setCurrentCurvature(float curvature) {
  computeMinMaxCurvatureRadius(curvature);
  current_curvature_ = curvature;
}

float CarObstacleDetection::getPhi() const {
  return phi_;
}

} // nav_utils