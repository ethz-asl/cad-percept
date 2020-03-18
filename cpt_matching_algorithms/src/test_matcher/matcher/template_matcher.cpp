#include "test_matcher/test_matcher.h"

namespace cad_percept {
namespace matching_algorithms {

void TestMatcher::template_match() {
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "            Template matcher started           " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "This is only a template matcher. It returns a translation in z direction."
            << std::endl;
  transform_TR[2] = -5.0;
}

}  // namespace matching_algorithms
}  // namespace cad_percept
