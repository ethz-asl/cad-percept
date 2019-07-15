// A struct reading all the mapper parameters from parameter server as simplification

#include <pointmatcher_ros/get_params_from_server.h>

#ifndef CPT_SELECTIVE_ICP_MAPPER_PARAMETERS_H_
#define CPT_SELECTIVE_ICP_MAPPER_PARAMETERS_H_

namespace cad_percept {
namespace cpt_selective_icp {

struct MapperParameters {

  MapperParameters();
  ~MapperParameters();

  // Parameters

  std::string scan_topic(getParam<std::string>("scanTopic", "fail"));
  std::string reference_mesh(getParam<std::string>("referenceMesh", "fail"));
  int min_reading_point_coun(getParam<int>("minReadingPointCount", 2000));
  int input_queue_size(getParam<int>("inputQueueSize", 10));
  int map_sampling_density(getParam<int>("mapSamplingDensity", 100)); // Points per square meter
};

}
}
#endif // CPT_SELECTIVE_ICP_MAPPER_PARAMETERS_H_