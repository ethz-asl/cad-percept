#include "test_Matcher/test_Matcher_parameters.h"
#include <pointmatcher_ros/get_params_from_server.h>

namespace cad_percept {
namespace cpt_matching_algorithms {

TestMatcherParameters::TestMatcherParameters()
    : cad_topic(getParam<std::string>("cadTopic", "fail")),
      input_queue_size(getParam<int>("inputQueueSize", 10)),
      map_sampling_density(getParam<int>("mapSamplingDensity", 100)),
      tf_map_frame(getParam<std::string>("tfMapFrame", "/map")) {}

TestMatcherParameters::~TestMatcherParameters() {}

}  // namespace cpt_matching_algorithms
}  // namespace cad_percept