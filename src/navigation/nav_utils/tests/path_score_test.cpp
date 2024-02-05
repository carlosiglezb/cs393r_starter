#include <gtest/gtest.h>

#include <path_score_manager.h>

static float err_tol = 1e-6;

namespace nav_utils {

TEST(LinearScoringTest, simpleAdd) {
  // initialize
  PathScoreManager path_score(1);
  float w_dtg = path_score.getWDtg();

  float score_res = path_score.computeScores(0.4, 0., 1.);
  float score_expected = 0.4 + w_dtg * 1.;
  ASSERT_TRUE(score_res - score_expected <= err_tol);
  float score_unexpected = 0.2 + w_dtg * 1.;
  ASSERT_FALSE(score_res - score_unexpected <= err_tol);
}

TEST(LinearScoringTest, simpleAddVector) {
  // initialize
  unsigned int vec_size = 5;
  PathScoreManager path_score(vec_size);

  float score_sum = 0.;
  for (unsigned int i = 0; i < vec_size; i++) {
    score_sum += path_score.computeScores(1., 2., 3.);
  }

  float score_expected = vec_size * path_score.computeScores(1., 2., 3.);
  ASSERT_TRUE(score_sum - score_expected <= err_tol);
  ASSERT_FALSE(score_sum - 2.*score_expected <= err_tol);
}

TEST(LinearScoringTest, maximumScoreIndex) {
  // initialize
  unsigned int vec_size = 5;
  PathScoreManager path_score(vec_size);
  path_score.setWClear(0.8);
  path_score.setWDtg(0.3);

  // sample parameters
  path_score.computeAndStore(1., 2., 3.);   // score: 1 + 1.6 + 0.9 = 3.5
  path_score.computeAndStore(4., 5., 6.);   // score: 4 + 4 + 1.8 = 9.8
  path_score.computeAndStore(1., 6., 3.);   // score: 1 + 4.8 + 0.9 = 6.7

  int max_score_idx = path_score.getMaximumScoreIdx();

  int max_idx_expected = 1;
  ASSERT_TRUE(max_score_idx == max_idx_expected);
  ASSERT_FALSE(max_score_idx == max_idx_expected - 1);
}

} // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}