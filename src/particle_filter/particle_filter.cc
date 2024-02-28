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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

#define b_DEBUG false

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

static double n_particles = 50;
DEFINE_double(num_particles, n_particles, "Number of particles");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    particles_(n_particles) {

  // Initialize to zero
  for (auto & sample : particles_) {
    sample.loc.setZero();
    sample.angle = 0.;
    sample.weight = 0.;
  }

  // tuning parameters of motion model
  k1_ = 1e-3;
  k2_ = 1e-3;
  k3_ = 1e-4;
  k4_ = 1e-4;
}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) {
    scan[i] = Vector2f(0, 0);
  }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  const Vector2f laser_loc(loc[0] + 0.2 * cos(angle), loc[1] + 0.2 * sin(angle));
  for (size_t i = 0; i < map_.lines.size(); ++i) {
    const line2f map_line = map_.lines[i];
    const float angle_increment = (angle_max - angle_min) / num_ranges;
    float laser_angle = angle_min;
    for (int i = 0; i < num_ranges; i++) {
      float laser_x = cos(laser_angle + angle);
      float laser_y = sin(laser_angle + angle);
      line2f laser_line(
        laser_loc[0] + range_min * laser_x,
        laser_loc[1] + range_min * laser_y,
        laser_loc[0] + range_max * laser_x,
        laser_loc[1] + range_max * laser_y);
      bool intersects = map_line.Intersects(laser_line);
      Vector2f intersection_point;
      intersects = map_line.Intersection(laser_line, &intersection_point);
      if (intersects) {
        scan[i] = intersection_point;
      } else {
        scan[i] = Vector2f(laser_line.p1.x(), laser_line.p1.y());
      }
      laser_angle += angle_increment;
    }
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
  Particle& particle = *p_ptr;
  int num_ranges = ranges.size();
  float gamma = 1.0;
  float sigma = 1.0;

  vector<Vector2f> predicted_scan;
  GetPredictedPointCloud(
          particle.loc,
          particle.angle,
          num_ranges,
          range_min,
          range_max,
          angle_min,
          angle_max,
          &predicted_scan);
  const Vector2f laser_loc(particle.loc[0] + 0.2 * cos(particle.angle), particle.loc[1] + 0.2 * sin(particle.angle));
  vector<float> likelihoods(num_ranges);
  for (int i = 0; i < num_ranges; i++) {
    float observed_range = ranges[i];
    Vector2f predicted_point = predicted_scan[i];
    float predicted_range = sqrt(pow(laser_loc[0] - predicted_point[0], 2) + pow(laser_loc[1] - predicted_point[1], 2));
    likelihoods[i] = pow((observed_range - predicted_range) / sigma, 2);
  }

  float log_likelihood = 0.0;
  for (float likelihood: likelihoods) {
    log_likelihood += likelihood;
  }
  particle.weight = -gamma * log_likelihood;
}

std::vector<Particle> ParticleFilter::resampleParticles(const std::vector<Particle>& particles) {
  double totalWeight = std::accumulate(particles.begin(), particles.end(), 0.0,
                                       [](double sum, const Particle& p) { return sum + p.weight; });

  std::vector<Particle> resampledParticles;
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0, totalWeight);

  for (size_t i = 0; i < particles.size(); ++i) {
    double x = distribution(generator);
    double wSum = 0;
    for (const auto& particle : particles) {
      wSum += particle.weight;
      if (wSum > x) {
        resampledParticles.push_back(particle);
        break;
      }
    }
  }

  return resampledParticles;
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  vector<Particle> new_particles = resampleParticles(particles_);
  particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  //  float x = rng_.UniformRandom(0, 1);
  //  printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
  //         x);
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.

  // loop over every particle and update its weight based on LiDAR observation
  for (auto &particle: particles_) {
    Update(ranges, range_min, range_max, angle_min, angle_max, &particle);
  }

  // [DEBUG]
  if (b_DEBUG) {
    Vector2f loc_sum(0., 0.);
    for (const auto &p: particles_) {
      loc_sum += p.loc;
    }
    loc_sum /= particles_.size();
    std::cout << "[Update] Location: (" << loc_sum.transpose() << ")" << std::endl;
  }

  // Resample particles based on their importance weights
  Resample();
}

float ParticleFilter::sampleNormal(const float k1,
                                   const float vel,
                                   const float k2,
                                   const float omega) {
  float std_dev = std::sqrt(k1 * std::abs(vel) + k2 * std::abs(omega));
  return rng_.Gaussian(0.0, std_dev);
}

Pose2Df ParticleFilter::sampleNextErrorState(const float thetat,
                                             const Eigen::Vector2f &delta_loc,
                                             const float delta_angle) {
  Pose2Df delta_pose;

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

void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.
  Vector2f delta_loc = odom_loc - prev_odom_loc_;
  float delta_angle = odom_angle - prev_odom_angle_;

  for (auto & sample : particles_) {
    // Compute the estimated change in pose for each particle
    // note: the orientation of each particle may be different
    Pose2Df error_pose = sampleNextErrorState(sample.angle, delta_loc, delta_angle);

    // Update the (estimated) particle's pose
    sample.loc += error_pose.translation;
    sample.angle += error_pose.angle;
  }

  // [DEBUG]
  if (b_DEBUG) {
    Vector2f loc_sum(0., 0.);
    for (const auto &p: particles_) {
      loc_sum += p.loc;
    }
    loc_sum /= particles_.size();
    std::cout << "[Predict] Location: (" << loc_sum.transpose() << ")" << std::endl;
  }

  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  //  float x = rng_.Gaussian(0.0, 2.0);
  //  printf("Random number drawn from Gaussian distribution with 0 mean and "
  //         "standard deviation of 2 : %f\n", x);
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);

  // TODO add randomness around loc and angle
  for (auto & sample : particles_) {
    sample.loc = loc;
    sample.angle = angle;
  }
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:

  if (!odom_initialized_) {
    loc(0., 0.);
    angle = 0.;
  }

  // compute average location and mean angle using atan2
  Vector2f loc_sum(0., 0.);
  float sin_theta_sum = 0.;
  float cos_theta_sum = 0.;
  for (size_t i = 0; i < particles_.size(); ++i) {
    loc_sum += particles_[i].loc;
    sin_theta_sum += std::sin(particles_[i].angle);
    cos_theta_sum += std::cos(particles_[i].angle);
  }
  loc = loc_sum / particles_.size();
  angle = std::atan2(sin_theta_sum / particles_.size(),
                     cos_theta_sum / particles_.size());
  if (b_DEBUG) {
    std::cout << "Location: (" << loc.transpose() << ")" << std::endl;
    std::cout << "Angle: (" << angle << ")" << std::endl;
  }

}


}  // namespace particle_filter
