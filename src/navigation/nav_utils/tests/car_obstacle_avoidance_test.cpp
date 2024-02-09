#include <gtest/gtest.h>

#include <car_obstacle_avoidance.h>

static float err_tol = 1e-6;

namespace nav_utils {

/**
 * Detect obstacle on left side, hence move right
 */
TEST(ObstacleAvoidanceTest, avoid_obstacle_on_left) {
  //Settings
  // Car measurements
  float car_w = 0.281;
  float car_l = 0.535;
  float car_wb = 0.5;

  // Number of paths to evaluate
  unsigned int n_paths = 7;

  // Point cloud data for testing
  std::vector<Eigen::Vector2f> object_points;

  // Create object (points) in world frame
  for (float pc_y = 0.3; pc_y > 0.0; pc_y -= 0.025) {
    object_points.emplace_back(1., pc_y);
  }
  // Create wall (points) in world frame behind obstacle and copy to local frame
  for (float pc_y = 0.0; pc_y > -0.3; pc_y -= 0.025) {
    object_points.emplace_back(4., pc_y);
  }
  std::vector<Eigen::Vector2f> object_points_base = object_points;
  std::vector<float> free_path_eval(object_points.size());

  // Create car obstacle detection
  CarObstacleAvoidance car_obs_avoid(car_w, car_l, car_wb,
                                     n_paths, object_points.size());
  float dt = car_obs_avoid.getControllerDt();

  // Compute free path length and optimal control (velocity) to apply
  float cmd_vel, optim_curve;
  Eigen::Vector2f p_goal(3., 0.);

  // Initialize
  float x = 0.;
  float y = 0.;
  float theta = 0.;
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

    // Do control
    Eigen::Vector3f twist_meas;
    twist_meas.x() = x;
    twist_meas.y() = y;
    twist_meas.z() = theta;
    car_obs_avoid.doControl(object_points_base, p_goal, twist_meas);

    // Update control
    cmd_vel = car_obs_avoid.getCmdVel();
    optim_curve = car_obs_avoid.getCmdCurvature();

    // Integration (kinematics)
    x += cmd_vel * std::cos(optim_curve) * dt;
    y += cmd_vel * std::sin(optim_curve) * dt;
    theta += cmd_vel * optim_curve * dt;

    // Check no collision near (3cm from) obstacle
    if (abs(x - 1) < 0.03) {
      ASSERT_TRUE(y < 0.05);
    }
  }
  Eigen::Vector2f end_car_pos(x, y);
  ASSERT_TRUE((p_goal - end_car_pos).norm() < 0.03);
  ASSERT_TRUE(abs(cmd_vel) < err_tol);
}

/**
 * Detect obstacle on right side, hence move left
 */
TEST(ObstacleAvoidanceTest, avoid_obstacle_on_right) {
  //Settings
  // Car measurements
  float car_w = 0.281;
  float car_l = 0.535;
  float car_wb = 0.5;

  // Number of paths to evaluate
  unsigned int n_paths = 15;

  // Point cloud data for testing
  std::vector<Eigen::Vector2f> object_points;

  // Create wall (points) in world frame behind obstacle
  for (float pc_y = 0.3; pc_y > -0.; pc_y -= 0.025) {
    object_points.emplace_back(4., pc_y);
  }
  // Create object (points) in world frame and copy to local frame
  for (float pc_y = -0.; pc_y > -0.3; pc_y -= 0.025) {
    object_points.emplace_back(1., pc_y);
  }
  std::vector<Eigen::Vector2f> object_points_base = object_points;
  std::vector<float> free_path_eval(object_points.size());

  // Create car obstacle detection
  std::shared_ptr<CarObstacleAvoidance> car_obs_avoid;

  car_obs_avoid = std::make_shared<CarObstacleAvoidance>(car_w, car_l, car_wb,
                                     n_paths, object_points.size());
  float dt = car_obs_avoid->getControllerDt();

  // Compute free path length and optimal control (velocity) to apply
  float cmd_vel, optim_curve;
  Eigen::Vector2f p_goal(3., 0.);

  // Initialize
  float x = 0.;
  float y = 0.;
  float theta = 0.;
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

    // Do control
    Eigen::Vector3f twist_meas;
    twist_meas.x() = x;
    twist_meas.y() = y;
    twist_meas.z() = theta;
    car_obs_avoid->doControl(object_points_base, p_goal, twist_meas);

    // Update control
    cmd_vel = car_obs_avoid->getCmdVel();
    optim_curve = car_obs_avoid->getCmdCurvature();

    // Integration (kinematics)
    x += cmd_vel * std::cos(optim_curve) * dt;
    y += cmd_vel * std::sin(optim_curve) * dt;
    theta += cmd_vel * optim_curve * dt;

    // Check no collision near (3cm from) obstacle
    if (abs(x - 1) < -0.03) {
      ASSERT_TRUE(y < 0.05);
    }
  }
  Eigen::Vector2f end_car_pos(x, y);
  ASSERT_TRUE((p_goal - end_car_pos).norm() < 0.05);
  ASSERT_TRUE(abs(cmd_vel) < err_tol);
}

/**
 * Detect obstacle in front tilted clockwise, hence move left
 */
TEST(ObstacleAvoidanceTest, avoid_obstacle_tilted_cw) {
      //Settings
      // Car measurements
      float car_w = 0.281;
      float car_l = 0.535;
      float car_wb = 0.5;

      // Number of paths to evaluate
      unsigned int n_paths = 15;

      // Point cloud data for testing
      std::vector<Eigen::Vector2f> object_points;

      // Create wall (points) in world frame behind obstacle
      for (float pc_y = 0.6; pc_y > 0.3; pc_y -= 0.025) {
        object_points.emplace_back(4., pc_y);
      }
      // Create object (points) in world frame and copy to local frame
      float pc_x = 0.;
      for (float pc_y = 0.1; pc_y > -0.3; pc_y -= 0.025) {
        object_points.emplace_back(1. - pc_x, pc_y);
        pc_x -= 0.02;     // add (cw) tilt
      }
      std::vector<Eigen::Vector2f> object_points_base = object_points;
      std::vector<float> free_path_eval(object_points.size());

      // Create car obstacle detection
      std::shared_ptr<CarObstacleAvoidance> car_obs_avoid;

      car_obs_avoid = std::make_shared<CarObstacleAvoidance>(car_w, car_l, car_wb,
                                                             n_paths, object_points.size());
      float dt = car_obs_avoid->getControllerDt();

      // Compute free path length and optimal control (velocity) to apply
      float cmd_vel, optim_curve;
      Eigen::Vector2f p_goal(3., 0.);

      // Initialize
      float x = 0.;
      float y = 0.;
      float theta = 0.;
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

        // Do control
        Eigen::Vector3f twist_meas;
        twist_meas.x() = x;
        twist_meas.y() = y;
        twist_meas.z() = theta;
        car_obs_avoid->doControl(object_points_base, p_goal, twist_meas);

        // Update control
        cmd_vel = car_obs_avoid->getCmdVel();
        optim_curve = car_obs_avoid->getCmdCurvature();

        // Integration (kinematics)
        x += cmd_vel * std::cos(optim_curve) * dt;
        y += cmd_vel * std::sin(optim_curve) * dt;
        theta += cmd_vel * optim_curve * dt;
      }
      Eigen::Vector2f end_car_pos(x, y);
      ASSERT_TRUE((p_goal - end_car_pos).norm() < 0.05);
      ASSERT_TRUE(abs(cmd_vel) < err_tol);
    }


} // nav_utils


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}