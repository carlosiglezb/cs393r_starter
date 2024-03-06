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

static double n_particles = 100;
DEFINE_double(num_particles, n_particles, "Number of particles");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    particles_(n_particles),
    prev_particles_(n_particles) {

  // Initialize to zero
  for (auto & sample : particles_) {
    sample.loc.setZero();
    sample.angle = 0.;
    sample.weight = 0.;
  }

  // Initialize to zero
  for (auto & sample : prev_particles_) {
    sample.loc.setZero();
    sample.angle = 0.;
    sample.weight = 0.;
  }
  max_laser_range_ = 10.; // [m]

  // tuning parameters of motion model
  k1_ = 8e-2;
  k2_ = 1e-1;
  k3_ = 1e-2;
  k4_ = 1e-2;

  laser_interval_ = 20;   // TODO figure out what to do with the weight of skipped scans
  gamma_ = 1.0;
  sigma_ = 0.2;
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

  const Vector2f laser_loc(loc[0] + 0.2 * std::cos(angle), loc[1] + 0.2 * std::sin(angle));
  const float angle_increment = (angle_max - angle_min) / num_ranges;
  float laser_angle = angle_min;
  for (int j = 0; j < num_ranges; j += laser_interval_) {
    float laser_x = std::cos(laser_angle + angle);
    float laser_y = std::sin(laser_angle + angle);
    line2f laser_line(
            laser_loc[0] + range_min * laser_x,
            laser_loc[1] + range_min * laser_y,
            laser_loc[0] + range_max * laser_x,
            laser_loc[1] + range_max * laser_y);
    float prev_scan_dist = max_laser_range_;
    for (size_t i = 0; i < map_.lines.size(); ++i) {
      const line2f map_line = map_.lines[i];
      bool intersects = map_line.Intersects(laser_line);
      Vector2f intersection_point;
      intersects = map_line.Intersection(laser_line, &intersection_point);

      // if current scan intersect this map line, save it in scan and proceed to next laser scan
      if (intersects) {
        float curr_scan_dist = (laser_loc - intersection_point).norm();
        // check and grab closest intersection when there are multiple intersections
        if (curr_scan_dist < prev_scan_dist) {
          scan[j] = intersection_point;
          prev_scan_dist = curr_scan_dist;    // update closest scan
        }
      } else {  // current scan does not intersect current map line

        // if current scan has not intersected previous map lines, assign max range in scan direction
        if (prev_scan_dist >= max_laser_range_ - 0.01) {
          scan[j] = Vector2f(laser_line.p1.x(), laser_line.p1.y());
        }
      }
    }
    laser_angle += angle_increment * laser_interval_;
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

  if (!odom_initialized_)
    return;

  int num_ranges = ranges.size();

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
  const Vector2f laser_loc(particle.loc[0] + 0.2 * std::cos(particle.angle), particle.loc[1] + 0.2 * std::sin(particle.angle));
  vector<float> likelihoods(num_ranges);
  for (int i = 0; i < num_ranges; i+= laser_interval_) {
    float observed_range = ranges[i];
    Vector2f predicted_point = predicted_scan[i];
    float predicted_range = sqrt(pow(laser_loc[0] - predicted_point[0], 2) + pow(laser_loc[1] - predicted_point[1], 2));
    likelihoods[i] = pow((observed_range - predicted_range) / sigma_, 2);
  }

  float log_likelihood = 0.0;
  for (float likelihood: likelihoods) {
    log_likelihood += likelihood;
  }
  particle.weight = -gamma_ * log_likelihood;
}

std::vector<Particle> ParticleFilter::resampleParticles(const std::vector<Particle>& particles) {
  double totalWeight = std::accumulate(particles.begin(), particles.end(), 0.0,
                                       [](double sum, const Particle& p) { return sum + p.weight; });

  std::vector<Particle> resampledParticles;
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0, totalWeight);

  for (size_t i = 0; i <= particles.size(); ++i) {
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
  // TODO below check not needed?
  if (!odom_initialized_)
    return;

  // Resample the particles, proportional to their weights.
  vector<Particle> new_particles = resampleParticles(particles_);
  particles_ = new_particles;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  if (!odom_initialized_)
    return;

  // loop over every particle and update its weight based on LiDAR observation
  for (auto &particle: particles_) {
    Update(ranges, range_min, range_max, angle_min, angle_max, &particle);
  }

  // Resample particles based on their importance weights
//  Resample();

  // [DEBUG]
  if (b_DEBUG) {
    Vector2f loc_sum(0., 0.);
    for (const auto &p: particles_) {
      loc_sum += p.loc;
    }
    loc_sum /= particles_.size();
    std::cout << "[Update] Location: (" << loc_sum.transpose() << ")" << std::endl;
  }
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
  if( !odom_initialized_ )
  {
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;
    return;
  }

  // Get change in position in odom frame
  Vector2f odom_delta_loc = odom_loc - prev_odom_loc_;
  Eigen::Matrix2f base_R_odom;
  base_R_odom << std::cos(-prev_odom_angle_), -std::sin(-prev_odom_angle_),
          std::sin(-prev_odom_angle_), std::cos(-prev_odom_angle_);
  Vector2f base_delta_pos = base_R_odom * odom_delta_loc;

  // Get change in angle in odom frame
  float base_delta_angle = odom_angle - prev_odom_angle_;
  if (b_DEBUG) {
    std::cout << "[Predict] (base_delta_pos, base_delta_angle): ("
              << base_delta_pos.transpose() << ", "
            << base_delta_angle << ")" << std::endl;
  }

  // Add uncertainty
  for (auto & sample : particles_) {
    // Compute the estimated change in pose for each particle
    // note: the orientation of each particle may be different
    Vector2f delta_pos_err(0., 0.);
    delta_pos_err.x() = sampleNormal(k1_, base_delta_pos.norm(), 0., base_delta_angle);
    delta_pos_err.y() = sampleNormal(k2_, base_delta_pos.norm(), 0., base_delta_angle);
    float delta_angle_err = sampleNormal(k3_, base_delta_pos.norm(), k4_, base_delta_angle);

    // Update the (estimated) particle's pose
    Eigen::Matrix2f map_R_base;
    map_R_base << std::cos(sample.angle), -std::sin(sample.angle),
            std::sin(sample.angle), std::cos(sample.angle);
    sample.loc += map_R_base * (base_delta_pos + delta_pos_err);
    sample.angle += (base_delta_angle + delta_angle_err);
  }

  // Find raw mean position at current time step
  Vector2f loc_sum(0., 0.);
  for (const auto &p: prev_particles_) {
    loc_sum += p.loc;
  }
  loc_sum /= prev_particles_.size();

  // Remove particles that collide with map
  for (size_t i = 0; i < particles_.size(); ++i) {
    const Vector2f& loc = particles_[i].loc;
    const Vector2f& prev_loc = prev_particles_[i].loc;

    // if particle has gone through a wall, resample from raw mean
    if (map_.Intersects(loc, prev_loc)) {

      // Resample error
      Vector2f delta_pos_err(0., 0.);
      delta_pos_err.x() = sampleNormal(k1_, base_delta_pos.norm(), 0., base_delta_angle);
      delta_pos_err.y() = sampleNormal(k2_, base_delta_pos.norm(), 0., base_delta_angle);

      Eigen::Matrix2f map_R_base;
      Particle sample = particles_[i];
      map_R_base << std::cos(sample.angle), -std::sin(sample.angle),
              std::sin(sample.angle), std::cos(sample.angle);

      // Get new estimated location for particle
      particles_[i].loc = loc_sum + map_R_base * (base_delta_pos + delta_pos_err);
    }
  }

  // Find corrected mean position at current time step
  loc_sum(0., 0.);
  for (const auto &p: particles_) {
    loc_sum += p.loc;
  }
  loc_sum /= particles_.size();
  // [DEBUG]
  if (b_DEBUG) {
    std::cout << "[Predict] Post_location: (" << loc_sum.transpose() << ")" << std::endl;
  }

  // set current odometry reading as previous for next loop
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
  prev_particles_ = particles_;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);
  odom_initialized_ = false;

  // Reset particles to zero
  for (auto & sample : particles_) {
    sample.loc.setZero();
    sample.angle = 0.;
    sample.weight = 0.;
  }

  // add uncertainty around initialized loc and angle
  for (auto & sample : particles_) {
    sample.loc.x() = loc.x() + rng_.Gaussian(0, 0.05);  // allow 5 cm error
    sample.loc.y() = loc.y() + rng_.Gaussian(0, 0.05);  // allow 5 cm error
    sample.angle = angle + rng_.Gaussian(0, 0.1);       // allow 6 deg error
  }
  // [DEBUG]
  if (b_DEBUG) {
    Vector2f loc_sum(0., 0.);
    for (const auto &p: particles_) {
      loc_sum += p.loc;
    }
    loc_sum /= particles_.size();
    std::cout << "============================= INITIALIZE ============================= " << std::endl;
    std::cout << "[Initialize] Location: (" << loc_sum.transpose() << ")" <<
              " w/" << particles_.size() << " particles" << std::endl;
  }

  // update previous odometry
  prev_odom_loc_ = loc;
  prev_odom_angle_ = angle;
  prev_particles_ = particles_;
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr,
                                 bool b_print) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:

  if (!odom_initialized_) {
    loc(0., 0.);
    angle = 0.;
    if (b_DEBUG) {
      std::cout << "Initialization not complete, yet" << std::endl;
    }
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
  if (b_DEBUG && b_print) {
    std::cout << "[GetLocation] (Location, Angle): (" << loc.transpose() << ", "
              << angle << ")" << std::endl;
  }

}


}  // namespace particle_filter
