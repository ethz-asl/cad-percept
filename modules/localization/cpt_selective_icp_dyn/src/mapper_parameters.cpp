#include <pointmatcher_ros/get_params_from_server.h>

#include "cpt_selective_icp_dyn/mapper_parameters.h"

namespace cad_percept {
namespace selective_icp_dyn {

MapperParameters::MapperParameters()
    : scan_topic(getParam<std::string>("scanTopic", "fail")),
      cad_topic(getParam<std::string>("cadTopic", "fail")),
      odom_topic(getParam<std::string>("odomTopic", "fail")),
      min_reading_point_count(getParam<int>("minReadingPointCount", 2000)),
      input_queue_size(getParam<int>("inputQueueSize", 10)),
      map_sampling_density(getParam<int>("mapSamplingDensity", 100)),  // Points per square meter
      tf_map_frame(getParam<std::string>("tfMapFrame", "/map")),
      lidar_frame(getParam<std::string>("lidarFrame", "lidar")),
      camera_pose_frame(getParam<std::string>("cameraPoseFrame", "camera_pose_frame")),
      camera_odom_frame(getParam<std::string>("cameraOdomFrame", "camera_odom_frame")),
      min_overlap(getParam<double>("minOverlap", 0.5)),
      mapping_trigger(getParam<bool>("mappingTrigger", false)),
      update_icp_ref_trigger(getParam<bool>("updateICPRefTrigger", false)),
      full_icp_primer_trigger(getParam<bool>("fullICPPrimerTrigger", false)),
      standalone_icp(getParam<bool>("standaloneICP", false)),
      path(getParam<std::string>("path", "fail")),
      output(getParam<bool>("output", false)),
      publish_distance(getParam<bool>("publishDistance", false)), 
      skip_scans(getParam<int>("skipScans", 1)),
      icp_cov_lin(getParam<double>("icpCovLin", 0.1)),
      icp_cov_rot(getParam<double>("icpCovRot", 0.1)),
      ekf_enable(getParam<bool>("ekfEnable", false)) {}
MapperParameters::~MapperParameters() {}

}  // namespace selective_icp_dyn
}  // namespace cad_percept
