#include <gtest/gtest.h>

#include <car_obstacle_detection.h>
#include <path_score_manager.h>
#include <curved_path.h>

static float err_tol = 1.e-6;

namespace nav_utils{

TEST(ObstacleDetectionTest, free_path_length_straight) {
  // Car measurements
  float car_w = 0.281;
  float car_l = 0.535;
  float car_wb = 0.5;

  // Point cloud data for testing
  Eigen::Vector2f point_cloud(1., 0.);

  // Create car obstacle detection
  CarObstacleDetection obstacle_detection(car_w, car_l, car_wb);

  // Check free path
  obstacle_detection.setCurrentCurvature(0.);
  float free_path = obstacle_detection.estimateFreePath(point_cloud);
  float expected_free_path = float(1.) - car_l;
  ASSERT_TRUE(free_path - expected_free_path < err_tol);
}

TEST(ObstacleDetectionTest, free_path_length_not_in_collision) {
  // Car measurements
  float car_w = 0.281;
  float car_l = 0.535;
  float car_wb = 0.5;
  float curvature = 0.011;

  // Point cloud data for testing
  Eigen::Vector2f point_cloud(1., 0.5);

  // Create car obstacle detection
  CarObstacleDetection obstacle_detection(car_w, car_l, car_wb);

  // Check free path
  obstacle_detection.setCurrentCurvature(curvature);
  float free_path = obstacle_detection.estimateFreePath(point_cloud);

  // Lazy check: compare length against straight line
  float free_path_underapprox = point_cloud.norm() - car_l;
  ASSERT_TRUE(free_path_underapprox < free_path);
}

TEST(ObstacleDetectionTest, free_path_length_left_turn) {
  // Car measurements
  float car_w = 0.281;
  float car_l = 0.535;
  float car_wb = 0.5;

  // Point cloud data for testing
  Eigen::Vector2f point_cloud(1., 0.3);

  // Lazy check: compare length against straight line
  Eigen::Vector2f front_car_to_p(point_cloud[0] - car_l, point_cloud[1]);
  float free_path_overapprox = front_car_to_p.norm();

  // Create car obstacle detection
  CarObstacleDetection obstacle_detection(car_w, car_l, car_wb);

  float free_path;
  // Check free path
  for(float curvature = 0.2; curvature > 0.1; curvature -= 0.01) {
    obstacle_detection.setCurrentCurvature(curvature);
    free_path = obstacle_detection.estimateFreePath(point_cloud);
    ASSERT_TRUE(free_path < free_path_overapprox );
  }
}

TEST(ObstacleDetectionTest, free_path_length_right_turn) {
  // Car measurements
  float car_w = 0.281;
  float car_l = 0.535;
  float car_wb = 0.5;

  // Point cloud data for testing
  Eigen::Vector2f point_cloud(1., -0.3);

  // Lazy check: compare length against straight line
  Eigen::Vector2f front_car_to_p(point_cloud[0] - car_l, point_cloud[1]);
  float free_path_overapprox = front_car_to_p.norm();

  // Create car obstacle detection
  CarObstacleDetection obstacle_detection(car_w, car_l, car_wb);

  float free_path;
  // Check free path
  for(float curvature = -0.2; curvature < -0.1; curvature += 0.01) {
    obstacle_detection.setCurrentCurvature(curvature);
    free_path = obstacle_detection.estimateFreePath(point_cloud);
    ASSERT_TRUE(free_path < free_path_overapprox );
  }
}

TEST(ObstacleDetectionTest, free_path_length_symmetric_left_right) {
  // Car measurements
  float car_w = 0.281;
  float car_l = 0.535;
  float car_wb = 0.5;
  float free_path_l, free_path_r;

  // Create car obstacle detection
  CarObstacleDetection obstacle_detection_l(car_w, car_l, car_wb);
  CarObstacleDetection obstacle_detection_r(car_w, car_l, car_wb);

  // Point cloud data for testing
  Eigen::Vector2f point_cloud_l(1., 0.);
  Eigen::Vector2f point_cloud_r(1., 0.);

  float curvature = 0.2;
  for(float delta_y = 0.3; delta_y > -0.1; delta_y -= 0.01) {
    point_cloud_l(1) = delta_y;
    point_cloud_r(1) = -delta_y;
    obstacle_detection_l.setCurrentCurvature(curvature);
    obstacle_detection_r.setCurrentCurvature(-curvature);
    free_path_l = obstacle_detection_l.estimateFreePath(point_cloud_l);
    free_path_r = obstacle_detection_r.estimateFreePath(point_cloud_r);
    ASSERT_TRUE(free_path_l - free_path_r < err_tol);   // check left/right solutions symmetric
    ASSERT_TRUE(free_path_l > 0.);    // check free path is positive when turning left
    ASSERT_TRUE(free_path_r > 0.);    // check free path is positive when turning right
  }
}

/**
 * Drive up to obstacle on left by combining Obstacle Detection and TOC
 */
TEST(ObstacleDetectionTest, move_up_to_obstacle_left) {
  // Car measurements
  float car_w = 0.281;
  float car_l = 0.535;
  float car_wb = 0.5;

  // Point cloud data for testing
  Eigen::Vector2f point_cloud(1., 0.3);
  Eigen::Vector2f point_cloud_local;

  // Compute free path length and optimal control (velocity) to apply
  float free_path, cmd_vel, dt;
  CurvedPath cpath_calc;

  // Create car obstacle detection
  CarObstacleDetection obstacle_detection(car_w, car_l, car_wb);

  // Run test for a range of curvatures turning left
  for (float curvature = 0.2; curvature > 0.1; curvature -= 0.01) {
    // Update curvature
    obstacle_detection.setCurrentCurvature(curvature);

    // Curved Path TOC Calculator
    cpath_calc = CurvedPath(curvature);
    dt =  cpath_calc.getDt();

    // Apply control action and integrate
    float x = 0.;
    float y = 0.;
    float theta =  0.;
    Eigen::Matrix2f c_R_w;    // rotation of car w.r.t. world
    Eigen::Vector2f w_p_car;  // position of car w.r.t. world
    for (unsigned int t = 0; t < 100; t ++) {
      // Update point cloud to reflect moving car
      w_p_car << x, y;
      c_R_w.row(0) << std::cos(-theta), -std::sin(-theta);
      c_R_w.row(1) << std::sin(-theta), std::cos(-theta);
      point_cloud_local = c_R_w * (point_cloud - w_p_car);

      // Update control
      free_path = obstacle_detection.estimateFreePath(point_cloud_local);
      cmd_vel = cpath_calc.compute1DTOC(free_path);

      // Integration (kinematics)
      x += cmd_vel * std::cos(curvature) * dt;
      y += cmd_vel * std::sin(curvature) * dt;
      theta += cmd_vel * curvature * dt;

      // Check
      ASSERT_TRUE(free_path >= 0.);    // check that free path is non-negative
      ASSERT_TRUE(theta >= 0.);    // check that car is actually turning left
    }

    // check that we arrived to the obstacle with zero velocity
    ASSERT_TRUE(std::abs(point_cloud_local.x() - car_l) < 0.02);
    ASSERT_TRUE(std::abs(cmd_vel) < err_tol);
    ASSERT_TRUE(std::abs(free_path) < 0.02);
  }
}

/**
 * Drive up to obstacle on right by combining Obstacle Detection and TOC
 */
TEST(ObstacleDetectionTest, move_up_to_obstacle_right) {
  // Car measurements
  float car_w = 0.281;
  float car_l = 0.535;
  float car_wb = 0.5;

  // Point cloud data for testing
  Eigen::Vector2f point_cloud(1., -0.3);
  Eigen::Vector2f point_cloud_local;

  // Compute free path length and optimal control (velocity) to apply
  float free_path, cmd_vel, dt;
  CurvedPath cpath_calc;

  // Create car obstacle detection
  CarObstacleDetection obstacle_detection(car_w, car_l, car_wb);

  // Run test for a range of curvatures turning left
  for (float curvature = -0.2; curvature < -0.1; curvature += 0.01) {
    // Update curvature
    obstacle_detection.setCurrentCurvature(curvature);

    // Curved Path TOC Calculator
    cpath_calc = CurvedPath(curvature);
    dt =  cpath_calc.getDt();

    // Apply control action and integrate
    float x = 0.;
    float y = 0.;
    float theta =  0.;
    Eigen::Matrix2f c_R_w;    // rotation of car w.r.t. world
    Eigen::Vector2f w_p_car;  // position of car w.r.t. world
    for (unsigned int t = 0; t < 100; t ++) {
      // Update point cloud to reflect moving car
      w_p_car << x, y;
      c_R_w.row(0) << std::cos(-theta), -std::sin(-theta);
      c_R_w.row(1) << std::sin(-theta), std::cos(-theta);
      point_cloud_local = c_R_w * (point_cloud - w_p_car);

      // Update control
      free_path = obstacle_detection.estimateFreePath(point_cloud_local);
      cmd_vel = cpath_calc.compute1DTOC(free_path);

      // Integration (kinematics)
      x += cmd_vel * std::cos(curvature) * dt;
      y += cmd_vel * std::sin(curvature) * dt;
      theta += cmd_vel * curvature * dt;

      // Check
      ASSERT_TRUE(free_path >= -0.03);    // check that free path is non-negative
      ASSERT_TRUE(theta <= 0.);    // check that car is actually turning right
    }

    // check that we arrived to the obstacle with zero velocity
    ASSERT_TRUE(std::abs(point_cloud_local.x() - car_l) < 0.06);
    ASSERT_TRUE(std::abs(cmd_vel) < err_tol);
    ASSERT_TRUE(std::abs(free_path) < 0.02);
  }
}

/**
 * Detect obstacle on left side, hence move right using scoring function
 */
TEST(ObstacleAvoidanceTest, avoid_obstacle_on_left) {
  //Settings
  float clearance = 0.1;
  // Car measurements
  float car_w = 0.281;
  float car_l = 0.535;
  float car_wb = 0.5;
  // Create car obstacle detection
  CarObstacleDetection car_obs_det(car_w, car_l, car_wb);

  // Create score manager for paths and path TOC calculator
  unsigned int n_paths = 7;
  std::vector<float> curvature_vec(n_paths);
  PathScoreManager score_mgr(n_paths);
  CurvedPath cpath_calc;
  float dt = cpath_calc.getDt();

  // Point cloud data for testing
  std::vector<Eigen::Vector2f> object_points;

  // Create object (points) in world frame
  for (float pc_y = 0.3; pc_y > 0.1; pc_y -= 0.025) {
    object_points.emplace_back(1., pc_y);
  }
  // Create wall (points) in world frame behind obstacle and copy to local frame
  for (float pc_y = 0.1; pc_y > -0.3; pc_y -= 0.025) {
    object_points.emplace_back(4., pc_y);
  }
  std::vector<Eigen::Vector2f> object_points_base = object_points;
  std::vector<float> free_path_eval(object_points.size());

  // Compute free path length and optimal control (velocity) to apply
  float min_free_path, cmd_vel, dist_to_goal;
  Eigen::Vector2f p_goal(3., 0.);
  Eigen::Vector2f aug_r(0., 0.);    // vector (0, r)

  // Loop over candidate curvatures
  float c_max = 0.2;
  float c_min = -0.2;
  float curvature_inc = (c_max - c_min) / n_paths;

  // Create vector of curvatures to loop through them and recall min curve
  for (unsigned int i = 0; i < n_paths; i++) {
    curvature_vec.push_back(c_max - i*curvature_inc);
  }

  // Initialize
  float x = 0.;
  float y = 0.;
  float theta =  0.;
  Eigen::Matrix2f c_R_w;    // rotation of car w.r.t. world
  Eigen::Vector2f w_p_car;  // position of car w.r.t. world

  for (unsigned int t = 0; t < 200; t++) {

    // Update point cloud in base frame to reflect moving car
    w_p_car << x, y;
    c_R_w.row(0) << std::cos(-theta), -std::sin(-theta);
    c_R_w.row(1) << std::sin(-theta), std::cos(-theta);
    for (unsigned int p_i = 0; p_i < object_points_base.size(); p_i++) {
      object_points_base[p_i] = c_R_w * (object_points[p_i] - w_p_car);
    }

    // Evaluate all candidate curves
    score_mgr.resetIdx();
//  for (float curvature = c_max; curvature > c_min; curvature -= curvature_inc) {
    for (auto &curvature: curvature_vec) {
      // Update curvature
      car_obs_det.setCurrentCurvature(curvature);
      aug_r(1) = float(1.) / curvature;
      dist_to_goal = (p_goal - aug_r).norm();   // TODO check < distance to goal from car

      // Evaluate score of all points
      for (unsigned int p_i = 0; p_i < object_points_base.size(); p_i++) {
        free_path_eval[p_i] = car_obs_det.estimateFreePath(object_points_base[p_i]);
        //TODO limit free path to radial distance
      }
      // the shortest path corresponds to the point that it will collide with first
      min_free_path = std::min_element(free_path_eval.begin(), free_path_eval.end()) - free_path_eval.begin();
      Eigen::Vector2f fp_end_location;
      float phi = min_free_path / aug_r(1);
      fp_end_location(0) = x + aug_r(1) * std::cos(phi);
      fp_end_location(1) = y + aug_r(1) * std::sin(phi);
      dist_to_goal = (p_goal - fp_end_location).norm();

      // Update score manager
      score_mgr.computeAndStore(min_free_path, clearance, dist_to_goal);

      // Add any important assertion for each curve here
    }

    // Choose path with max score and apply control
    unsigned int max_score_idx = score_mgr.getMaximumScoreIdx();

    // Update control
    cmd_vel = cpath_calc.compute1DTOC(free_path_eval[max_score_idx]);

    // Integration (kinematics)
    x += cmd_vel * std::cos(curvature_vec[max_score_idx]) * dt;
    y += cmd_vel * std::sin(curvature_vec[max_score_idx]) * dt;
    theta += cmd_vel * curvature_vec[max_score_idx] * dt;
  }
  Eigen::Vector2f end_car_pos(x, y);
  ASSERT_TRUE((p_goal - end_car_pos).norm() < err_tol);
}

} // nav_utils



int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}