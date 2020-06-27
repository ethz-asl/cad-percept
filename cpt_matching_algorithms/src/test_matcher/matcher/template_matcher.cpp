#include "test_matcher/test_matcher.h"

namespace cad_percept {
namespace matching_algorithms {

void TestMatcher::templateMatch() {
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "            Template matcher started           " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "This is only a template matcher. It returns the Identity." << std::endl;

  res_transform_ = Eigen::Matrix4d::Identity();
}

}  // namespace matching_algorithms
}  // namespace cad_percept
