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
  ///
  /// \brief Constructs a new MabiLocalizer object.
  ///
  /// \param nh                         Node handle.
  /// \param nh_private                 Private node handle.
  /// \param reference_link_topic_name  Topic name of a reference link that has fixed pose w.r.t. to
  ///    the lasers.
  /// \param end_effector_topic_name    Topic name of the end effector.
  MabiLocalizer(ros::NodeHandle &nh, ros::NodeHandle &nh_private,
                std::string reference_link_topic_name = "gripper",
                std::string end_effector_topic_name = "end_effector");
  bool highAccuracyLocalization(
      cpt_pointlaser_loc_ros::HighAccuracyLocalization::Request &request,
      cpt_pointlaser_loc_ros::HighAccuracyLocalization::Response &response);

 private:
  void advertiseTopics();
  kindr::minimal::QuatTransformation getTF(std::string from, std::string to);
  ///
  /// \brief Sets the arm (end effector) to a certain pose, where the pose is w.r.t. the robot base.
  ///
  /// \param arm_goal_pose  Goal pose of the arm end effector w.r.t. robot base.
  void setArmTo(const kindr::minimal::QuatTransformation &arm_goal_pose);
  void setTaskType(const std_msgs::Int16 &task_type_msg);
  ///
  /// \brief Reads from an input message the offset pose - w.r.t. the world frame - to which the arm
  ///    should initially be moved.
  /// \param msg  Message containing the offset pose.
  ///
  void getOffsetPose(const geometry_msgs::PoseStamped::ConstPtr &msg);

  // Reference model.
  cad_percept::cgal::MeshModel::Ptr model_;
  // Node handles.
  ros::NodeHandle nh_, nh_private_;
  // Publisher, subscribers.
  // - Publishers of the intersections of the lasers with the model, for debug purposes.
  ros::Publisher pub_intersection_a_, pub_intersection_b_, pub_intersection_c_;
  // - Publisher of the pose from marker to end-effector.
  ros::Publisher pub_endeffector_pose_;
  // - Subscriber to task mode:
  //   - Task mode 3 = HAL.
  ros::Subscriber sub_mode_;
  // - Subscriber to task type: each valid task type is associated to a movement defined as
  //   "movement_type_<task_type>" on the parameter server.
  ros::Subscriber sub_task_type_;
  // - Subscriber to the initial pose of the arm in the world frame.
  ros::Subscriber sub_offset_pose_;
  // Transform listener.
  tf::TransformListener transform_listener_;
  // Service clients and server.
  std::map<std::string, ros::ServiceClient> leica_client_, mabi_client_;
  ros::ServiceServer high_acc_localisation_service_;
  // Internal parameters.
  std::string reference_link_topic_name_, end_effector_topic_name_;
  Eigen::Matrix<double, 6, 1> initial_armbase_to_ref_link_std_, odometry_noise_std_;
  double pointlaser_noise_std_;
  int task_type_;
  bool transform_received_;
  kindr::minimal::QuatTransformation initial_world_to_arm_pose_;
  // Localizer that performs the high-accuracy localization task.
  std::unique_ptr<cad_percept::pointlaser_loc::localizer::PointLaserLocalizer> localizer_;
};
}  // namespace pointlaser_loc_ros
}  // namespace cad_percept
#endif
