// A struct reading all the mapper parameters from parameter server as simplification
#ifndef CPT_SELECTIVE_ICP_DYN_MAPPER_PARAMETERS_H_
#define CPT_SELECTIVE_ICP_DYN_MAPPER_PARAMETERS_H_

namespace cad_percept {
namespace selective_icp_dyn {

struct MapperParameters {
  MapperParameters();
  ~MapperParameters();

  // Parameters
  std::string scan_topic;
  std::string cad_topic;
  std::string odom_topic;
  int min_reading_point_count;
  int input_queue_size;
  int map_sampling_density;  // Points per square meter
  std::string tf_map_frame;
  std::string lidar_frame;
  std::string camera_odom_frame;
  std::string camera_pose_frame;
  double min_overlap;
  bool mapping_trigger;
  bool update_icp_ref_trigger;
  bool full_icp_primer_trigger;
  bool standalone_icp;
  std::string path;
  bool output;
  bool publish_distance;
  int skip_scans;
  bool ekf_enable;
  double icp_cov_lin;
  double icp_cov_rot;
};

}  // namespace selective_icp
}  // namespace cad_percept
#endif  // CPT_SELECTIVE_ICP_DYN_MAPPER_PARAMETERS_H_
