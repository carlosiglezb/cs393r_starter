//
// Created by carlos-pc on 2/3/24.
//

#ifndef NAV_UTILS_CAR_OBSTACLE_DETECTION_HPP
#define NAV_UTILS_CAR_OBSTACLE_DETECTION_HPP

#include <vector>
#include <memory>
#include <eigen3/Eigen/Core>

#include "curved_path.h"

namespace nav_utils {

class CarObstacleDetection {
public:
    CarObstacleDetection(float car_width,
                         float car_length,
                         float car_wheelbase,
                         float car_height=0.15);
    ~CarObstacleDetection();

    void computeMinMaxCurvatureRadius(float curvature);

    /**
     * Estimates the free path of the car before reaching point
     * cloud point_cloud_2D.
     * Note: Currently, assumes:
     *  (1) current curvature
     *  (2) point cloud hits the front of the car
     * @param point_cloud_2D (x,y) of point cloud data
     * @return free path length
     */
    float estimateFreePath(const Eigen::Vector2f& point_cloud_2D);

    float freePathToClosestPoA(const Eigen::Vector2f &w_p_goal,
                               const Eigen::Vector2f &w_p_car,
                               const float theta);

    bool isInCollision() const;

    // Setters
    void setCurrentCurvature(float curvature);

    // Getters
    float getPhi() const;
private:
    // Min radius that can hit a point, given fixed radius of curvatures
    float r_min_;
    // Max radius that can hit a point, given fixed radius of curvatures
    float r_max_;
    // Current curvature, c = 1/r
    float current_curvature_;
    // Is current point cloud in collision
    bool b_p_in_collision_;
    // Angle of path
    float phi_;
    // Angle of path trimmed by closest point of approach
    float phi_poa_;

    // Paths TODO not being used, move to Obstacle Avoidance
//    std::vector<std::shared_ptr<CurvedPath>> paths_;

    // Car dimensions
    float car_width_;
    float car_length_;
    float car_wheelbase_;
    float car_height_;

    // Curvature assumed to be zero for numerical purposes
    float curvature_zero_thresh_;
    // Minimum turn radius
    float min_turn_radius_;
};

} // nav_utils

#endif //NAV_UTILS_CAR_OBSTACLE_DETECTION_HPP
