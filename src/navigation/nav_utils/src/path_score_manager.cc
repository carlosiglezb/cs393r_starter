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
            scores_(n_paths),
            clearances_(n_paths),
            dists_to_goal_(n_paths),
            fp_dist_(n_paths) {
      w_clear_ = 0.2;
      w_dtg_ = -4000.;
      idx_ = 0;
    }

    PathScoreManager::~PathScoreManager() {}

    void PathScoreManager::computeAndStore(float path_length,
                                           float clearance,
                                           float distance_to_goal) {
      scores_[idx_] = computeScores(path_length, clearance, distance_to_goal);
      dists_to_goal_[idx_] = distance_to_goal;
      clearances_[idx_] = clearance;
      saveFreePaths(path_length);
      idx_++;
    }

    void PathScoreManager::saveFreePaths(const float free_path) {
      fp_dist_[idx_] = free_path;
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

    void PathScoreManager::reset() {
      for (auto & score : scores_) {
        score = 0.;
      }
      for (auto & dist : dists_to_goal_) {
        dist = 0.;
      }
      for (auto & fp : fp_dist_) {
        fp = 0.;
      }
      for (auto & cl : clearances_) {
        cl = 0.;
      }
      resetIdx();
    }

    void PathScoreManager::resetIdx() {
      idx_ = 0;
    }

    float PathScoreManager::getClearance(const unsigned int path_idx) {
      return clearances_[path_idx];
    }

    float PathScoreManager::getWDtg() const {
      return w_dtg_;
    }

    float PathScoreManager::getWClear() const {
      return w_clear_;
    }

    float PathScoreManager::getDistanceToGoal(const unsigned int idx) const {
      return dists_to_goal_[idx];
    }

    float PathScoreManager::getFpDistance(const unsigned int idx) const {
      return fp_dist_[idx];
    }

    void PathScoreManager::setWDtg(float new_w_dtg) {
      this->w_dtg_ = new_w_dtg;
    }

    void PathScoreManager::setWClear(float new_w_clear) {
      this->w_clear_ = new_w_clear;
    }

} // path_score_calculator