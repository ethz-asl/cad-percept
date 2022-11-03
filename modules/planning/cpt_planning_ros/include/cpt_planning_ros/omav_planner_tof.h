//
// Created by mpantic on 03.11.22.
//

#ifndef CPT_PLANNING_ROS_OMAV_PLANNER_TOF_H
#define CPT_PLANNING_ROS_OMAV_PLANNER_TOF_H
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <rmpcpp/eval/trapezoidal_integrator.h>
#include <rmpcpp/eval/trapezoidal_integrator_so3.h>
#include <rmpcpp/geometry/linear_geometry.h>
#include <rmpcpp/policies/rotation_target_policy.h>
#include <rmpcpp/geometry/rotated_geometry_3d.h>
#include <rmpcpp/policies/simple_target_policy.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>

class OmavPlanner {
  using R3Geometry = rmpcpp::LinearGeometry<3>;
  using R3Space = rmpcpp::Space<3>;
  using R3GeometryPtr = std::shared_ptr<rmpcpp::LinearGeometry<3>>;
  using R3TargetPolicy = rmpcpp::SimpleTargetPolicy<R3Space>;
  using R3TargetPolicyPtr = std::shared_ptr<R3TargetPolicy>;
  using R3Policies = std::vector<std::shared_ptr<rmpcpp::PolicyBase<R3Space>>>;
  using R3Integrator = rmpcpp::TrapezoidalIntegrator<rmpcpp::PolicyBase<R3Space>, rmpcpp:: GeometryBase<3,3>>;
  using R3RotatedGeometry = rmpcpp::RotatedGeometry3d;
  using R3RotatedGeometryPtr =  std::shared_ptr<rmpcpp::RotatedGeometry3d>;
  using SO3Geometry = rmpcpp::LinearGeometry<4>;                      // hack
  using SO3GeometryPtr = std::shared_ptr<rmpcpp::LinearGeometry<4>>;  // hack
  using SO3Space = rmpcpp::QuaternionSpace;
  using SO3TargetPolicy = rmpcpp::RotationTargetPolicy;
  using SO3TargetPolicyPtr = std::shared_ptr<SO3TargetPolicy>;
  using SO3Policies = std::vector<std::shared_ptr<rmpcpp::PolicyBase<SO3Space>>>;
  using So3Integrator = rmpcpp::TrapezoidalIntegratorSO3<rmpcpp::PolicyBase<SO3Space>, SO3Geometry>;

 public:
  OmavPlanner(ros::NodeHandle nh, ros::NodeHandle nh_private);
  OmavPlanner() = delete;

  void presenterCallback(const geometry_msgs::Vector3StampedPtr& vctr);
  void normalCallback(const geometry_msgs::PoseStampedPtr& vctr);
  void setPointCallback(const trajectory_msgs::MultiDOFJointTrajectoryPtr& traj);
  void setWeights(double pos_global, double pos_z, double pos_body);
  void joystickCallback(const sensor_msgs::JoyConstPtr &joy);
  void runOrientationPlanner(mav_msgs::EigenTrajectoryPointVector* traject);
 void runPositionPlanner(mav_msgs::EigenTrajectoryPointVector * traject);
  bool debugCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  void run();

 private:
  R3TargetPolicyPtr pol_global_;  // target policy in global frame
  R3TargetPolicyPtr
      pol_cam_z_;  // z-holding policy assumption: distance is roughly in z axis of camera
  R3TargetPolicyPtr pol_body_;          // target xy policy in body frame
  R3TargetPolicyPtr pol_body_ptr_;          // target xy policy in body frame for pointer
  SO3TargetPolicyPtr pol_orientation_;  // orientation holding policy

  bool state_valid_{false};
  Eigen::Affine3d T_world_odom_{Eigen::Affine3d::Identity()};
  Eigen::Affine3d T_odom_body_{Eigen::Affine3d::Identity()};  // transform from body to global
  Eigen::Affine3d T_body_cam_{Eigen::Affine3d::Identity()};   // transform from cam to body
  Eigen::Vector3d v_, w_;        // velocity in odom, and body velocities
  Eigen::Vector3d vdot_, wdot_;  // acc

  Eigen::Quaterniond desired_normal_orientation_;

  double dt_{0.005};                  // [s] temporal resolutions for trajectories
  double max_traject_duration_{2.0};  // [s] amount of lookahead for integration


  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber sub_curr_ref_;
  ros::Publisher pub_setpoint_;
  ros::Subscriber sub_joystick_;
  ros::Subscriber sub_pose_;
  ros::Subscriber sub_presenter_;
  Eigen::Vector3d ptr_target_;
  int num_targets_ptr_{0};

  ros::ServiceServer debug_trigger_;
};

#endif  // CPT_PLANNING_ROS_OMAV_PLANNER_TOF_H
