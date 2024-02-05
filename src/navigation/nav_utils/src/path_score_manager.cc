/**
 * Keeps track of the scores obtained from the linear weighting:
 *
 *  score = free path length + w_1 * clearance + w_2 * distance to goal
 *
 * and returns the index containing the maximum score, reflecting the
 * entry with the highest chance of succeeding collision-free navigation
 */

#include "path_score_manager.h"

namespace nav_utils {
    PathScoreManager::PathScoreManager(unsigned int n_paths) :
            scores_(n_paths) {
      w_clear_ = 0.1;
      w_dtg_ = -0.5;
      idx_ = 0;
    }

    PathScoreManager::~PathScoreManager() {}

    void PathScoreManager::computeAndStore(float path_length,
                                           float clearance,
                                           float distance_to_goal) {
      scores_[idx_] = computeScores(path_length, clearance, distance_to_goal);
      idx_++;
    }

    float PathScoreManager::computeScores(float path_length,
                                            float clearance,
                                            float distance_to_goal) const {
      return path_length + w_clear_ * clearance + w_dtg_ * distance_to_goal;
    }

    int PathScoreManager::getMaximumScoreIdx() const {
      int maxElementIndex = std::max_element(scores_.begin(),scores_.end()) - scores_.begin();
      return maxElementIndex;
    }

    void PathScoreManager::resetIdx() {
      idx_ = 0;
    }

    float PathScoreManager::getWDtg() const {
      return w_dtg_;
    }

    float PathScoreManager::getWClear() const {
      return w_clear_;
    }

    void PathScoreManager::setWDtg(float new_w_dtg) {
      this->w_dtg_ = new_w_dtg;
    }

    void PathScoreManager::setWClear(float new_w_clear) {
      this->w_clear_ = new_w_clear;
    }

} // path_score_calculator