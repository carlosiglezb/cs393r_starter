#include <gtest/gtest.h>

#include <motion_model_sampler.h>
#include "sciplot/sciplot.hpp"

using namespace sciplot;

namespace pf_utils {

TEST(MotionModelTest, sampleNextStatesFromOrigin) {
  // initialize
  unsigned int n_samples = 500;
  float dt = 0.05;
  float curvature = 0.5;
  MotionModelSampler motionSampler(0.324, curvature, dt, n_samples);

  // inputs
  float lin_vel = 1.;
  float ang_vel = lin_vel * curvature;
  Pose2Df current_state(0., 0., 0.);
  Eigen::Vector2f u_t_plus_1(lin_vel, ang_vel);   // v, w
  motionSampler.initialize(current_state);

  // variables to plot
  std::vector<float> origin_x, origin_y;
  std::vector<float> x_traj, y_traj;

  // generate M particles from single starting point
  for (unsigned int m = 0; m < n_samples; m++) {
    Pose2Df next_state = motionSampler.sampleNextState(current_state, u_t_plus_1);
    x_traj.push_back(next_state.translation.x());
    y_traj.push_back(next_state.translation.y());
    origin_x.push_back(0.);
    origin_y.push_back(0.);
  }

  // plot
  Plot2D plot_xy_pos;
  plot_xy_pos.xlabel("x");
  plot_xy_pos.ylabel("y");
  plot_xy_pos.drawDots(origin_x, origin_y).label("x_{0}").lineColor("red");
  plot_xy_pos.drawDots(x_traj, y_traj).label("x_{1}").lineColor("green").fillSolid();
  plot_xy_pos.legend().atTopRight();
  plot_xy_pos.xrange(-0.1, 0.2);
  plot_xy_pos.yrange(-0.1, 0.1);
  plot_xy_pos.grid();

  // Create figure to hold
  Figure fig = {{plot_xy_pos}};

  // Create canvas to hold figure
  Canvas canvas = {{fig}};
  canvas.size(750, 750);

  // Show the plot_tau_cmd in a pop-up window
  canvas.show();

  ASSERT_TRUE(true);
}

TEST(MotionModelTest, predictSamplesFromOrigin) {
  // initialize
  unsigned int n_samples = 500;
  float dt = 0.05;
  float curvature = -0.8;
  MotionModelSampler motionSampler(0.324, curvature, dt, n_samples);

  // inputs
  float lin_vel = 1.;
  float ang_vel = lin_vel * curvature;
  Pose2Df current_state(0., 0., 0.);
  Eigen::Vector2f u_t_plus_1(lin_vel, ang_vel);   // v, w
  motionSampler.initialize(current_state);

  // variables to plot
  std::vector<float> origin_x, origin_y;
  std::vector<float> x_traj, y_traj;

  // apply control input 4 times, increasing curvature each time
  for (unsigned int t = 0; t < 4; t++) {
    // update curvature (control)
    motionSampler.updateCurvature(curvature);

    // update control input with new curvature
    ang_vel = lin_vel * curvature;
    u_t_plus_1(lin_vel, ang_vel);

    // predict next particles
    motionSampler.predictParticles(u_t_plus_1);
    for (auto & sample : motionSampler.getParticles()) {
      x_traj.push_back(sample.translation.x());
      y_traj.push_back(sample.translation.y());
      origin_x.push_back(0.);
      origin_y.push_back(0.);
    }
    curvature += 0.4;
  }

  // plot
  Plot2D plot_xy_pos;
  plot_xy_pos.xlabel("x");
  plot_xy_pos.ylabel("y");
  plot_xy_pos.drawDots(origin_x, origin_y).label("x_{0}").lineColor("red");
  plot_xy_pos.drawDots(x_traj, y_traj).label("x_{t+1}").lineColor("green");
  plot_xy_pos.legend().atTopRight();
  plot_xy_pos.xrange(-0.1, 0.3);
  plot_xy_pos.yrange(-0.1, 0.1);
  plot_xy_pos.grid();

  // Create figure to hold
  Figure fig = {{plot_xy_pos}};

  // Create canvas to hold figure
  Canvas canvas = {{fig}};
  canvas.size(750, 750);

  // Show the plot_tau_cmd in a pop-up window
  canvas.show();

  ASSERT_TRUE(true);
}

TEST(MotionModelTest, predictSamplesMovingUp) {
  // initialize
  unsigned int n_samples = 500;
  float dt = 0.05;
  float curvature = -0.8;
  MotionModelSampler motionSampler(0.324, curvature, dt, n_samples);

  // inputs
  float lin_vel = 1.;
  float ang_vel = lin_vel * curvature;
  Pose2Df current_state(M_PI_2, 0., 0.);
  Eigen::Vector2f u_t_plus_1(lin_vel, ang_vel);   // v, w
  motionSampler.initialize(current_state);

  // variables to plot
  std::vector<float> origin_x, origin_y;
  std::vector<float> x_traj, y_traj;

  // apply control input 4 times, increasing curvature each time
  for (unsigned int t = 0; t < 4; t++) {
    // update curvature (control)
    motionSampler.updateCurvature(curvature);

    // update control input with new curvature
    ang_vel = lin_vel * curvature;
    u_t_plus_1 << lin_vel, ang_vel;

    // predict next particles
    motionSampler.predictParticles(u_t_plus_1);
    for (auto & sample : motionSampler.getParticles()) {
      x_traj.push_back(sample.translation.x());
      y_traj.push_back(sample.translation.y());
      origin_x.push_back(0.);
      origin_y.push_back(0.);
    }
    curvature += 0.6;
  }

  // plot
  Plot2D plot_xy_pos;
  plot_xy_pos.xlabel("x");
  plot_xy_pos.ylabel("y");
  plot_xy_pos.drawDots(origin_x, origin_y).label("x_{0}").lineColor("red");
  plot_xy_pos.drawDots(x_traj, y_traj).label("x_{t+1}").lineColor("green");
  plot_xy_pos.legend().atTopRight();
  plot_xy_pos.xrange(-0.1, 0.1);
  plot_xy_pos.yrange(-0.02, 0.3);
  plot_xy_pos.grid();

  // Create figure to hold
  Figure fig = {{plot_xy_pos}};

  // Create canvas to hold figure
  Canvas canvas = {{fig}};
  canvas.size(750, 750);

  // Show the plot_tau_cmd in a pop-up window
  canvas.show();

  ASSERT_TRUE(true);
}

} // namespaceP

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}