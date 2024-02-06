//
// Created by Carlos on 2/4/24.
//

#ifndef NAV_UTILS_CAR_OBSTACLE_AVOIDANCE_H
#define NAV_UTILS_CAR_OBSTACLE_AVOIDANCE_H

#include <memory>
#include "car_obstacle_detection.h"
#include "path_score_manager.h"
#include "curved_path.h"

namespace nav_utils {

class CarObstacleAvoidance {
public:
    CarObstacleAvoidance(float car_width,
                         float car_length,
                         float car_wheelbase,
                         unsigned int n_paths,
                         unsigned int n_scan_points,
                         float car_height=0.15);
    ~CarObstacleAvoidance();

    void doControl(const std::vector<Eigen::Vector2f> &point_cloud,
                   const Eigen::Vector2f &w_p_goal);

    // Getters
    float getCmdVel() const;
    float getCmdCurvature() const;
    float getControllerDt() const;

private:
    std::shared_ptr<CarObstacleDetection> obstacle_detection_;
    std::shared_ptr<PathScoreManager> score_mgr_;
    std::shared_ptr<CurvedPath> toc_controller_;

    // Free paths associated to each point cloud data point
    std::vector<float> free_paths_pc_vec_;

    // Estimate of robot states
    Eigen::Vector2f w_p_car_;
    float theta_;

    // Vector with candidate curvatures
    std::vector<float> candidate_curvatures_;
    // Commanded velocity output by the controller
    float cmd_vel_;
    // Commanded curvature output by the controller
    float cmd_curvature_;
    // For curve under evaluation, its effective free path before collision
    float curve_collision_free_fp_;
    // Minimum curvature to consider for local paths
    float c_min_;
    // Maximum curvature to consider for local paths
    float c_max_;
};

} // nav_utils

#endif //NAV_UTILS_CAR_OBSTACLE_AVOIDANCE_H
