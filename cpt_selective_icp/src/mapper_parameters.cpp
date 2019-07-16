#include <pointmatcher_ros/get_params_from_server.h>

#include "cpt_selective_icp/mapper_parameters.h"

namespace cad_percept {
namespace selective_icp {

MapperParameters::MapperParameters() :
  scan_topic(getParam<std::string>("scanTopic", "fail")),
  reference_mesh(getParam<std::string>("referenceMesh", "fail")),
  min_reading_point_coun(getParam<int>("minReadingPointCount", 2000)),
  input_queue_size(getParam<int>("inputQueueSize", 10)),
  map_sampling_density(getParam<int>("mapSamplingDensity", 100)) // Points per square meter
{}
MapperParameters::~MapperParameters() {}

} 
}
