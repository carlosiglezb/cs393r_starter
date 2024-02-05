//
// Created by carlos-pc on 2/2/24.
//

#ifndef NAV_UTILS_CURVED_PATH_H
#define NAV_UTILS_CURVED_PATH_H

#include <algorithm>

namespace nav_utils {
class CurvedPath {  //TODO rename to CarTOC1dCalculator ?
public:
  explicit CurvedPath(float curvature=0.);
  ~CurvedPath();
  float compute1DTOC(float dist_to_goal);

  // Getters
  const float getCurvature() const;
  const float getVelocity() const;
  const float getFreeLength() const;
  const float getDistToGoal() const;
  const float getDt() const;

  // Setters
  void setCurvature(float curvature);
  void setDistanceToGoal(float distance);
private:
  float curvature_;
  float velocity_cmd_;
  float free_path_length_;
  float dist_to_goal_;

  // Limits
  float max_vel_;   // max velocity
  float max_acc_;   // max acceleration
  float max_dec_;   // max deceleration
  float dt_;        // sampling time

  // Dealing with uncertainty
  float allowed_max_vel_err_;
  float allowed_max_pos_err_;
};

} // nav_utils
#endif //NAV_UTILS_CURVED_PATH_H
