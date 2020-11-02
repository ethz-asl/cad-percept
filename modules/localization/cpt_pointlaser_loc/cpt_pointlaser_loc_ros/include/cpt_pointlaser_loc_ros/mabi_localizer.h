#ifndef CPT_POINTLASER_LOC_ROS_MABI_LOCALIZER_H_
#define CPT_POINTLASER_LOC_ROS_MABI_LOCALIZER_H_

#include <cgal_definitions/mesh_model.h>
#include <cpt_pointlaser_loc/localizer/localizer.h>
#include <kindr/minimal/quat-transformation.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <tf/transform_listener.h>

#include <Eigen/Geometry>

#include "cpt_pointlaser_loc_ros/HighAccuracyLocalization.h"

namespace cad_percept {
namespace pointlaser_loc_ros {

/// \brief Class to control and localize the MABI arm.
class MabiLocalizer {
 public:
  MabiLocalizer(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

 private:
  cad_percept::cgal::MeshModel::Ptr model_;
  ros::NodeHandle nh_, nh_private_;
  tf::TransformListener transform_listener_;
  Eigen::Matrix<double, 6, 1> initial_pose_std_, odometry_noise_std_;
  double pointlaser_noise_std_;
  int mode_, task_type_;
  bool processing_, transform_received_;

  // Localizer that performs the high-accuracy localization task.
  std::unique_ptr<cad_percept::pointlaser_loc::localizer::PointLaserLocalizer> localizer_;
};
}  // namespace pointlaser_loc_ros
}  // namespace cad_percept
#endif
