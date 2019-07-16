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
  std::string reference_mesh;
  int min_reading_point_coun;
  int input_queue_size;
  int map_sampling_density; // Points per square meter
};

}
}
#endif // CPT_SELECTIVE_ICP_MAPPER_PARAMETERS_H_