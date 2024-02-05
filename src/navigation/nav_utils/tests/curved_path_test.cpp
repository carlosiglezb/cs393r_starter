#include <gtest/gtest.h>

#include <curved_path.h>

//static float err_tol = 1e-6;
namespace nav_utils {

TEST(CurvedPathTest, toc_1D_line) {
  CurvedPath path;
  path.setDistanceToGoal(1.);

  float vel_init = path.getVelocity();
  float vel_new_acc;
  // accelerate for 0.5 sec
  for (unsigned int t = 0; t < 20; t++) {
    vel_new_acc = path.compute1DTOC(1.2);
    ASSERT_TRUE(vel_init <= vel_new_acc);  // must accelerate
  }
  float vel_new_dec = path.compute1DTOC(0.12);
  ASSERT_TRUE(vel_new_acc > vel_new_dec);  // must decelerate
}

TEST(CurvedPathTest, start_to_stop_motion) {
  CurvedPath path;
  float dt = path.getDt();
  float goal = 1.;  // 1 m to goal

  float cmd_vel;
  float new_goal = goal;
  float x = 0;
  for (unsigned int t = 0; t < 60; t++) {
    new_goal = goal - x;
    // compute control
    cmd_vel = path.compute1DTOC(new_goal);

    // integrate
    x += cmd_vel * dt;
  }
  ASSERT_TRUE( std::abs(x - goal) < 0.01);
  ASSERT_TRUE(std::abs(cmd_vel) < 0.001);
}

} // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}