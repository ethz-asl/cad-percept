#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "cpt_deviation_analysis/deviations.h"

using namespace cad_percept::deviations;

TEST(DeviationsTest, aa_quat_trafo) {
  Eigen::Vector3d axis(1.5, 2, 0.5);
  axis.normalize();
  Eigen::AngleAxisd aa(0.5, axis);
  Eigen::Quaterniond quat(aa);
  Eigen::AngleAxisd compaa(quat);
  EXPECT_TRUE(abs(aa.angle() - compaa.angle()) < 1e-10);
  EXPECT_TRUE(aa.axis().isApprox(compaa.axis()));
}
