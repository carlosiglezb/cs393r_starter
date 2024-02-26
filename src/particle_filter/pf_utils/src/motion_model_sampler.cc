//
// Created by Carlos on 2/18/24.
//

#include "motion_model_sampler.h"

using namespace pose_2d;

namespace pf_utils {

MotionModelSampler::MotionModelSampler(float wheel_base,
                                       float curvature,
                                       float delta_t,
                                       unsigned int n_samples) :
        particles_(n_samples) {
  wheel_base_ = wheel_base;
  curvature_ = curvature;
  delta_t_ = delta_t;
  n_samples_ = n_samples;

  // tuning parameters
  k1_ = 1e-4;
  k2_ = 1e-2;
  k3_ = 1e-6;
  k4_ = 1e-6;
}

MotionModelSampler::~MotionModelSampler() = default;

void MotionModelSampler::initialize(const Eigen::Vector2f &loc,
                                    const float angle) {
  // TODO add randomness around loc and angle
  for (auto & sample : particles_) {
    sample.translation = loc;
    sample.angle = angle;
  }
}

void MotionModelSampler::predictParticles(const Eigen::Vector2f &current_control) {
  for (auto & sample : particles_) {
    sample = sampleNextState(sample, current_control);
  }
}

void MotionModelSampler::predictParticles(const Eigen::Vector2f &delta_loc,
                                          const float delta_angle) {
  for (auto & sample : particles_) {
    // Compute the estimated change in pose for each particle
    // note: the orientation of each particle may be different
    Pose2Df error_pose = sampleNextErrorState(sample, delta_loc, delta_angle);

    // Update the (estimated) particle's pose
    sample.translation += error_pose.translation;
    sample.angle += error_pose.angle;
  }
}

Pose2Df MotionModelSampler::sampleNextErrorState(const Pose2Df &x_t,
                                                 const Eigen::Vector2f &delta_loc,
                                                 const float delta_angle) {
  Pose2Df delta_pose;
  // simple change of variables for better bookkeeping
  float thetat = x_t.angle;

  // noise/uncertainty parameters
  Eigen::Vector2f car_eps_loc;    // car translation error in car frame

  car_eps_loc.x() = sampleNormal(k1_, delta_loc.norm(), 0., delta_angle);
  car_eps_loc.y() = sampleNormal(k2_, delta_loc.norm(), 0., delta_angle);
  float eps_theta = sampleNormal(k3_, delta_loc.norm(), k4_, delta_angle);

  // Rotation matrix from odom to car (base) frame, different for e/particle
  Eigen::Matrix2f odom_R_car;
  odom_R_car << std::cos(thetat), -std::sin(thetat),
          std::sin(thetat), std::cos(thetat);

  // Convert car translation error to odom frame and add to pose error
  delta_pose.translation = delta_loc + odom_R_car * car_eps_loc;
  delta_pose.angle = delta_angle + eps_theta;

  return delta_pose;
}

Pose2Df MotionModelSampler::sampleNextState(const Pose2Df &x_t,
                                            const Eigen::Vector2f &u_t_plus_1) {
  Pose2Df x_t_plus_1;
  // simple change of variables for better bookkeeping
  float v_t = u_t_plus_1.x();
  float w_t = u_t_plus_1.y();
  float xt = x_t.translation.x();
  float yt = x_t.translation.y();
  float thetat = x_t.angle;

  // intermediate parameters
  float steering_angle = std::atan(wheel_base_ * curvature_);
  Eigen::Vector2f c_vel;        // car velocity in car (base) frame
  Eigen::Vector2f c_eps_vel;    // car velocity error in car (base) frame

  c_vel.x() = v_t * std::cos(steering_angle);
  c_vel.y() = v_t * std::sin(steering_angle);
  c_eps_vel.x() = sampleNormal(k1_, c_vel.norm(), 0., w_t);
  c_eps_vel.y() = sampleNormal(0., c_vel.norm(), k2_, w_t);
  float eps_theta = sampleNormal(k3_, c_vel.norm(), k4_, w_t);

  // Rotation matrix from world to car (base) frame, different for e/particle
  Eigen::Matrix2f w_R_car;
  w_R_car << std::cos(thetat), -std::sin(thetat),
          std::sin(thetat), std::cos(thetat);

  // Convert car velocity error to world frame
  Eigen::Vector2f w_eps_vel = w_R_car * c_eps_vel;
  // Convert car velocity to world frame
  Eigen::Vector2f w_vel = w_R_car * c_vel;

  x_t_plus_1.translation.x() = xt + w_vel.x() * delta_t_ + w_eps_vel.x();
  x_t_plus_1.translation.y() = yt + w_vel.y() * delta_t_ + w_eps_vel.y();
  x_t_plus_1.angle = thetat + w_t * delta_t_ + eps_theta;

  return x_t_plus_1;
}

void MotionModelSampler::updateCurvature(const float curvature) {
  curvature_ = curvature;
}

Pose2Df MotionModelSampler::sampleNextStateFromVelModel(const Pose2Df &x_t,
                                            const Eigen::Vector2f &u_t_plus_1) {
  Pose2Df x_t_plus_1;
  // simple change of variables for better bookkeeping
  float v_t = u_t_plus_1.x();
  float w_t = u_t_plus_1.y();
  float xt = x_t.translation.x();
  float yt = x_t.translation.y();
  float thetat = x_t.angle;

  // intermediate parameters
  float v_hat, omega_hat, gamma_hat;

  // perturb commanded control parameters by noise drawn from kinematic model
  v_hat = v_t + sampleNormal(k1_, v_t, 0., w_t);
  omega_hat = w_t + sampleNormal(0., v_t, k2_, w_t);
  gamma_hat = sampleNormal(k3_, v_t, k4_, w_t);

  // generate sample's new pose
  float v_w_hat = v_hat / omega_hat;
  float wt_dt = omega_hat * delta_t_;
  x_t_plus_1.translation.x() =  xt - v_w_hat * std::sin(thetat)
          + v_w_hat * std::sin(thetat + wt_dt);
  x_t_plus_1.translation.y() =  yt + v_w_hat * std::cos(thetat)
          - v_w_hat * std::cos(thetat + wt_dt);
  x_t_plus_1.angle = thetat + wt_dt + gamma_hat * delta_t_;

  // intermediate parameters
  float steering_angle = std::atan(wheel_base_ * curvature_);
  Eigen::Vector2f vel;
  vel.x() = v_t * std::cos(thetat + steering_angle);
  vel.y() = v_t * std::sin(thetat + steering_angle);
  float eps_x = sampleNormal(k1_, vel.norm(), 0., w_t);
  float eps_y = sampleNormal(0., vel.norm(), k2_, w_t);
  float eps_theta = sampleNormal(k3_, vel.norm(), k4_, w_t);

  x_t_plus_1.translation.x() = xt + vel.x() * delta_t_ + eps_x;
  x_t_plus_1.translation.y() = yt + vel.y() * delta_t_ + eps_y;
  x_t_plus_1.angle = thetat + w_t * delta_t_ + eps_theta;

  return x_t_plus_1;
}

float MotionModelSampler::sampleNormal(const float k1,
                                       const float vel,
                                       const float k2,
                                       const float omega) {
  float std_dev = std::sqrt(k1 * std::abs(vel) + k2 * std::abs(omega));
  std::normal_distribution<float> normal_dist(0, std_dev);
  return normal_dist(generator_);
}

std::vector<Pose2Df> MotionModelSampler::getParticles() {
  return particles_;
}

} // pf_utils