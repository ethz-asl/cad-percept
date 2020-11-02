#include "cpt_pointlaser_loc_ros/mabi_localizer.h"

#include <geometry_msgs/PointStamped.h>
#include <kindr/minimal/position.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include "any_msgs/SetPose.h"
#include "cpt_pointlaser_comm_ros/GetDistance.h"

namespace cad_percept {
namespace pointlaser_loc_ros {

MabiLocalizer::MabiLocalizer(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      transform_listener_(nh_),
      mode_(0),
      task_type_(0),
      processing_(false),
      transform_received_(false) {
  if (!nh_private_.hasParam("off_model")) {
    ROS_ERROR("'off_model' not set as parameter.\n");
  }
  cad_percept::cgal::MeshModel::create(nh_private_.param<std::string>("off_model", "fail"), &model_,
                                       nh_private_.param("verbose", false));

  if (!nh_private_.hasParam("initial_pose_std"))
    ROS_ERROR("'initial_pose_std' not set as parameter.\n");
  initial_pose_std_ = Eigen::Matrix<double, 6, 1>(
      nh_private_
          .param<std::vector<double>>("initial_pose_std", std::vector<double>{1, 1, 1, 1, 1, 1})
          .data());
  if (!nh_private_.hasParam("arm_odometry_std"))
    ROS_ERROR("'arm_odometry_std' not set as parameter.\n");
  odometry_noise_std_ = Eigen::Matrix<double, 6, 1>(
      nh_private_
          .param<std::vector<double>>("arm_odometry_std", std::vector<double>{1, 1, 1, 1, 1, 1})
          .data());
  if (!nh_private_.hasParam("pointlaser_noise_std"))
    ROS_ERROR("'pointlaser_noise_std' not set as parameter.\n");
  pointlaser_noise_std_ = nh_private_.param<double>("pointlaser_noise_std", 1.0);

  // Initialize localizer.
  localizer_.reset(new cad_percept::pointlaser_loc::localizer::PointLaserLocalizer(
      model_, initial_pose_std_, odometry_noise_std_, pointlaser_noise_std_));

  advertiseTopics();
}

}  // namespace pointlaser_loc_ros
}  // namespace cad_percept