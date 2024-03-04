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
\file    particle-filter.h
\brief   Particle Filter Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/math/line2d.h"
#include "../shared/util/random.h"
#include "vector_map/vector_map.h"
#include "../shared/math/poses_2d.h"
#include <memory>

#ifndef SRC_PARTICLE_FILTER_H_
#define SRC_PARTICLE_FILTER_H_

using namespace pose_2d;

namespace particle_filter {

struct Particle {
  Eigen::Vector2f loc;
  float angle;
  double weight;
};

class ParticleFilter {
 public:
  // Default Constructor.
   ParticleFilter();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Method to sample next state given odometry errors
  Pose2Df sampleNextErrorState(const float thetat,
                               const Eigen::Vector2f &delta_loc,
                               const float delta_angle);

  // Predict particle motion based on odometry.
  void Predict(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Initialize the robot location.
  void Initialize(const std::string& map_file,
                  const Eigen::Vector2f& loc,
                  const float angle);

  // Return the list of particles.
  void GetParticles(std::vector<Particle>* particles) const;

  // Get robot's current location.
  void GetLocation(Eigen::Vector2f* loc, float* angle, bool b_print=false) const;

  // Update particle weight based on laser.
  void Update(const std::vector<float>& ranges,
              float range_min,
              float range_max,
              float angle_min,
              float angle_max,
              Particle* p);

    /**
   * Resamples particles based on their weights.
   *
   * @param particles The vector of particles to be resampled.
   * @return The resampled vector of particles.
   */
    std::vector<Particle> resampleParticles(const std::vector<Particle>& particles);

    // Resample particles.
  void Resample();

  // For debugging: get predicted point cloud from current location.
  void GetPredictedPointCloud(const Eigen::Vector2f& loc,
                              const float angle,
                              int num_ranges,
                              float range_min,
                              float range_max,
                              float angle_min,
                              float angle_max,
                              std::vector<Eigen::Vector2f>* scan);

private:
  float sampleNormal(const float k1,
                     const float vel,
                     const float k2,
                     const float omega);

private:
  // Map of the environment.
  vector_map::VectorMap map_;

  // Random number generator.
  util_random::Random rng_;

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  // List of particles being tracked.
  std::vector<Particle> particles_;
  // Model (tuning) uncertainty parameters
  float k1_;    // translational (x) error from translational movement
  float k2_;    // rotational error from translational movement
  float k3_;    // translational error from translational movement
  float k4_;    // rotational error from rotational movement

  int laser_interval_;
  float gamma_;
  float sigma_;
};
}  // namespace slam

#endif   // SRC_PARTICLE_FILTER_H_
