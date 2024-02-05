//
// Created by carlos-pc on 2/4/24.
//

#include "car_obstacle_avoidance.hpp"

namespace nav_utils {
    CarObstacleAvoidance::CarObstacleAvoidance(float car_width,
                                               float car_length,
                                               float car_wheelbase,
                                               unsigned int n_paths,
                                               float car_height) {
      obstacle_detection_ = std::make_shared<CarObstacleDetection>(car_width, car_length, car_wheelbase, car_height);
      score_mgr_ = std::make_shared<PathScoreManager>(n_paths);
//      paths_ = std::make_shared<CurvedPath>(0.);
    }

    CarObstacleAvoidance::~CarObstacleAvoidance() {

    }
} // nav_utils