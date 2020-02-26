#ifndef TEST_MATCHER_PARAMETERS_H_
#define TEST_MATCHER_PARAMETERS_H_

#include <string>  //Why necessary? Should be somewhere already be included

namespace cad_percept {
namespace cpt_matching_algorithms {

struct TestMatcherParameters {
  TestMatcherParameters();
  ~TestMatcherParameters();

  // Parameters
  std::string cad_topic;
  int input_queue_size;
  int map_sampling_density;
  std::string tf_map_frame;
};

}  // namespace cpt_matching_algorithms
}  // namespace cad_percept

#endif