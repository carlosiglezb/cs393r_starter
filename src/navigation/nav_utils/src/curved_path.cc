//
// Created by carlos-pc on 2/2/24.
//

#include <iostream>
#include "curved_path.h"

namespace nav_utils {

CurvedPath::CurvedPath(float curvature) {
  curvature_ = curvature;
  velocity_cmd_ = 0.;
  free_path_length_ = 0.;
  dist_to_goal_ = 0.;

  // Robot limits
  max_vel_ = 1.;    // 1 m/s
  max_acc_ = 4.;    // 4 m/s/s
  max_dec_ = -4.;   // -4 m/s/s
  dt_ = 0.025;    // 40 hz TODO check actual value

  allowed_max_vel_err_ = 0.01;
  allowed_max_pos_err_ = 0.01;  // 1 cm off is fine
}

CurvedPath::~CurvedPath() {}

float CurvedPath::compute1DTOC(float dist_to_goal) {

  // compute distance traversed if we apply max deceleration
  float dist_breaks_on = -velocity_cmd_ * velocity_cmd_ / (2. * max_dec_);
  dist_breaks_on += allowed_max_pos_err_;   // be a bit conservative TODO replace with safety margin

  // TODO add latency compensation or option to keep vel constant
  if ((velocity_cmd_ < max_vel_) && (dist_breaks_on < dist_to_goal) && (std::abs(dist_to_goal) > allowed_max_pos_err_)) {
    velocity_cmd_ += max_acc_ * dt_;                // accelerate
    velocity_cmd_ = std::min(velocity_cmd_, max_vel_);  // saturate
  } else if (velocity_cmd_ > (max_vel_ - allowed_max_vel_err_) && (dist_breaks_on < dist_to_goal)) {
    velocity_cmd_ = max_vel_;
  } else if (dist_breaks_on > dist_to_goal) {
    velocity_cmd_ += max_dec_ * dt_;                // decelerate
    velocity_cmd_ = std::max(velocity_cmd_, float(0.));  // saturate
  } else {
    velocity_cmd_ = 0;  // we've arrived
  }

  return velocity_cmd_;
}

const float CurvedPath::getCurvature() const {
  return curvature_;
}

const float CurvedPath::getVelocity() const {
  return velocity_cmd_;
}

const float CurvedPath::getFreeLength() const {
  return free_path_length_;
}

const float CurvedPath::getDistToGoal() const {
  return dist_to_goal_;
}

const float CurvedPath::getDt() const {
  return dt_;
}

void CurvedPath::setDistanceToGoal(float distance) {
  dist_to_goal_ = distance;
}

void CurvedPath::setCurvature(float curvature) {
  curvature_ = curvature;
}

} // nav_utils