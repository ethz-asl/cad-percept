#ifndef CPT_POINTLASER_LOC_ROS_KINOVA_LOCALIZER_H_
#define CPT_POINTLASER_LOC_ROS_KINOVA_LOCALIZER_H_

#include <cgal_definitions/mesh_model.h>
#include <cpt_pointlaser_loc/localizer/localizer.h>
#include <cpt_pointlaser_msgs/HighAccuracyLocalizationKinova.h>
#include <kindr/minimal/quat-transformation.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <tf/transform_listener.h>

#include <Eigen/Geometry>

namespace cad_percept {
namespace pointlaser_loc_ros {

/// \brief Class to control and localize the Kinova arm.
class KinovaLocalizer {
 public:
  KinovaLocalizer(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
  bool highAccuracyLocalization(
      cpt_pointlaser_msgs::HighAccuracyLocalizationKinova::Request &request,
      cpt_pointlaser_msgs::HighAccuracyLocalizationKinova::Response &response);

 private:
  void advertiseTopics();
  void setArmTo(const kindr::minimal::QuatTransformation &arm_goal_pose);
  void setMode(const std_msgs::Int16 &mode_msg);
  void setTaskType(const std_msgs::Int16 &task_type_msg);
  void getOffsetPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
  cad_percept::cgal::MeshModel::Ptr model_;
  ros::NodeHandle nh_, nh_private_;
  ros::Publisher pub_intersection_a_, pub_intersection_b_, pub_intersection_c_,
      pub_endeffector_pose_;
  ros::Subscriber sub_mode_, sub_task_type_, sub_offset_pose_;
  std::map<std::string, ros::ServiceClient> leica_client_, waco_client_;
  Eigen::Matrix<double, 6, 1> initial_pose_std_, odometry_noise_std_;
  double pointlaser_noise_std_;
  ros::ServiceServer high_acc_localisation_service_;
  int mode_, task_type_;
  bool processing_, transform_received_;
  kindr::minimal::QuatTransformation initial_arm_pose_;
  tf::TransformListener transform_listener_;
  // Localizer that performs the high-accuracy localization task.
  std::unique_ptr<cad_percept::pointlaser_loc::localizer::PointLaserLocalizer> localizer_;
};
}  // namespace pointlaser_loc_ros
}  // namespace cad_percept
#endif
