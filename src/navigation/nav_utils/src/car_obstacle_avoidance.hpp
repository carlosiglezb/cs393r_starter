//
// Created by Carlos on 2/4/24.
//

#ifndef NAV_UTILS_CAR_OBSTACLE_AVOIDANCE_HPP
#define NAV_UTILS_CAR_OBSTACLE_AVOIDANCE_HPP

#include <memory>
#include <car_obstacle_detection.h>
#include <path_score_manager.h>
#include <curved_path.h>

namespace nav_utils {

class CarObstacleAvoidance {
public:
    CarObstacleAvoidance(float car_width,
                         float car_length,
                         float car_wheelbase,
                         unsigned int n_paths,
                         float car_height=0.15);
    ~CarObstacleAvoidance();

private:
    std::shared_ptr<CarObstacleDetection> obstacle_detection_;
    std::shared_ptr<PathScoreManager> score_mgr_;
//    std::vector<std::shared_ptr<CurvedPath>> paths_;
};

} // nav_utils

#endif //NAV_UTILS_CAR_OBSTACLE_AVOIDANCE_HPP
