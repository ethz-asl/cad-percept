#include "test_Matcher/test_Matcher.h"

namespace cad_percept {
namespace matching_algorithms {

void test_Matcher::match(float (&transformTR)[6]) {
  std::cout << "This is only a template matcher. It returns a translation in z direction."
            << std::endl;
  transformTR[2] = 10.0;
}

}  // namespace matching_algorithms
}  // namespace cad_percept
