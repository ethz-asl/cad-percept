#include <pointmatcher_ros/get_params_from_server.h>

#include "cpt_selective_icp_dyn/mapper_parameters.h"

namespace cad_percept {
namespace selective_icp_dyn {

MapperParameters::MapperParameters()
    : scan_topic(getParam<std::string>("scanTopic", "fail")),
      cad_topic(getParam<std::string>("cadTopic", "fail")),
      min_reading_point_count(getParam<int>("minReadingPointCount", 2000)),
      input_queue_size(getParam<int>("inputQueueSize", 10)),
      map_sampling_density(getParam<int>("mapSamplingDensity", 100)),  // Points per square meter
      tf_map_frame(getParam<std::string>("tfMapFrame", "/map")),
      lidar_frame(getParam<std::string>("lidarFrame", "lidar")),
      min_overlap(getParam<double>("minOverlap", 0.5)),
      mapping_trigger(getParam<bool>("mappingTrigger", false)),
      update_icp_ref_trigger(getParam<bool>("updateICPRefTrigger", false)),
      full_icp_primer_trigger(getParam<bool>("fullICPPrimerTrigger", false)),
      standalone_icp(getParam<bool>("standaloneICP", false)),
      path(getParam<std::string>("path", "fail")),
      output(getParam<bool>("output", false)),
      publish_distance(getParam<bool>("publishDistance", false)) {}
MapperParameters::~MapperParameters() {}

}  // namespace selective_icp_dyn
}  // namespace cad_percept
