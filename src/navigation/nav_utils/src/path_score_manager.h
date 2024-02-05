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
  int getMaximumScoreIdx() const;
  void resetIdx();    // TODO add general reset (and reset scores as well)?

  // Getters
  float getWDtg() const;
  float getWClear() const;

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
  // index counter
  unsigned int idx_;

};
} // nav_utils

#endif //PATH_SCORE_MANAGER_HPP
