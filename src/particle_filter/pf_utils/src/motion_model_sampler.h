//
// Created by Carlos on 2/18/24.
//

#ifndef NAV_UTILS_MOTION_MODEL_HPP
#define NAV_UTILS_MOTION_MODEL_HPP

#include "../../../shared/math/poses_2d.h"
#include <random>

using namespace pose_2d;
namespace pf_utils {

class MotionModelSampler {
public:
    MotionModelSampler(float wheel_base,
                       float curvature,
                       float delta_t,
                       unsigned int n_samples);
    ~MotionModelSampler();

    /**
     * Initialize all particles in the particle filter with the current pose.
     * @param curent_pose
     */
    void initialize(const Eigen::Vector2f &loc, const float angle);

    /**
     * Predict the next state of all particles by applying to each the current_control.
     * @param current_control Vector of control commands containing the linear and
     * angular velocities (v, w)
     */
    void predictParticles(const Eigen::Vector2f &current_control);
    void predictParticles(const Eigen::Vector2f &delta_location, const float delta_angle);

    /**
     * Samples a new pose for each particle using a translation/rotation error model with
     * errors drawn from a zero mean, normal distribution with variance parameterized by
     * the linear and angular velocity magnitudes.
     * @param x_t   Current pose (x, y, theta)
     * @param u_t_plus_1    Control action (v, w)
     * @return Next sampled pose (x, y, theta)
     */
    Pose2Df sampleNextState(const Pose2Df &x_t,
                            const Eigen::Vector2f &u_t_plus_1);

    /**
     * Samples a new pose for each particle using a translation/rotation error model with
     * errors drawn from a zero mean, normal distribution with variance parameterized by
     * the linear and angular velocity magnitudes.
     * @param x_t Particle's pose
     * @param delta_location Change in (x,y) position in previous local coordinates
     * @param delta_angle Change in steering angle in previous local coordiantes
     * @return
     */
    Pose2Df sampleNextErrorState(const Pose2Df &x_t,
                                 const Eigen::Vector2f &delta_location,
                                 const float delta_angle);

    /**
     * Update the curvature (control) used of the motion prediction model.
     * @param curvature
     */
    void updateCurvature(const float curvature);

    /**
     * Implementation of velocity motion model for sampling prediction.
     * Note: currently not being used as it has edge case when
     * angular velocity != 0.
     * @param x_t   Current pose (x, y, theta)
     * @param u_t_plus_1    Control action (v, w)
     * @return Next sampled pose
     */
    Pose2Df sampleNextStateFromVelModel(const Pose2Df &x_t,
                            const Eigen::Vector2f &u_t_plus_1);

    /**
     * Get the set of particles at the current/latest time.
     * @return std::vector<Pose2Df> containing the pose of all particles.
     */
    std::vector<Pose2Df> getParticles();

private:
    float sampleNormal(const float k1,
                       const float vel,
                       const float k2,
                       const float omega);

private:
    // length of car wheelbase
    float wheel_base_;
    // fixed curvature (1/radius of curvature)
    float curvature_;
    // control update rate
    float delta_t_;
    // Number of samples used by the particle filter
    unsigned int n_samples_;

    // Model (tuning) uncertainty parameters
    float k1_;    // translational (x) error from translational movement
    float k2_;    // rotational error from translational movement
    float k3_;    // translational error from translational movement
    float k4_;    // rotational error from rotational movement

    // tools to generate random numbers
    std::default_random_engine generator_;
    // set of particles
    std::vector<Pose2Df> particles_;
};

} // fp_utils

#endif //NAV_UTILS_MOTION_MODEL_HPP
