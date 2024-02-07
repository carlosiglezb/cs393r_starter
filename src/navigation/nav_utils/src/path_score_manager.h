//
// Created by Carlos on 2/1/24.
//

#ifndef PATH_SCORE_MANAGER_HPP
#define PATH_SCORE_MANAGER_HPP

#include <vector>
#include <algorithm>

namespace nav_utils {
class PathScoreManager {
public:
  PathScoreManager(unsigned int n_paths);
  ~PathScoreManager();

  void computeAndStore(float path_length, float clearance, float distance_to_goal);
  float computeScores(float path_length, float clearance, float distance_to_goal) const;
  void saveFreePaths(const float free_path_dist_to_goal);
  int getMaximumScoreIdx() const;
  void resetIdx();    // TODO add general reset (and reset scores as well)?

  // Getters
  float getWDtg() const;
  float getWClear() const;
  float getDistanceToGoal(const unsigned int idx) const;
  float getFpDistance(const unsigned int idx) const;

  // Setters
  void setWDtg(float new_w_dtg);
  void setWClear(float new_w_clear);

private:
  // clearance weight
  float w_clear_;
  // distance to  weight
  float w_dtg_;
  // overall scores
  std::vector<float> scores_;
  // distances to goal corresponding to scores
  std::vector<float> dists_to_goal_;
  // Free path distances before hitting object or reaching PoA
  std::vector<float> fp_dist_;
  // index counter
  unsigned int idx_;

};
} // nav_utils

#endif //PATH_SCORE_MANAGER_HPP
