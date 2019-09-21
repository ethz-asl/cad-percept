// A struct reading all the mapper parameters from parameter server as simplification
#ifndef CPT_SELECTIVE_ICP_MAPPER_PARAMETERS_H_
#define CPT_SELECTIVE_ICP_MAPPER_PARAMETERS_H_

namespace cad_percept {
namespace selective_icp {

struct MapperParameters {
  MapperParameters();
  ~MapperParameters();

  // Parameters

  std::string scan_topic;
  std::string cad_topic;
  int min_reading_point_count;
  int input_queue_size;
  int map_sampling_density;  // Points per square meter
  std::string tf_map_frame;
  std::string lidar_frame;
  double min_overlap;
  bool mapping_trigger;
  bool update_icp_ref_trigger;
  bool normal_icp_primer_trigger;
  std::string path;
  bool output;
};

}  // namespace selective_icp
}  // namespace cad_percept
#endif  // CPT_SELECTIVE_ICP_MAPPER_PARAMETERS_H_
