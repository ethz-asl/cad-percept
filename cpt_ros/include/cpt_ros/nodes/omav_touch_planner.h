
#ifndef CPT_ROS_INCLUDE_CPT_ROS_NODES_OMAV_TOUCH_PLANNER_H_
#define CPT_ROS_INCLUDE_CPT_ROS_NODES_OMAV_TOUCH_PLANNER_H_

#include <cpt_planning/omav/surface_planner.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <cpt_planning/coordinates/face_coords.h>
#include <mav_msgs/conversions.h>

namespace cad_percept {

class OmavTouchPlanner {
 public:
  OmavTouchPlanner(ros::NodeHandle nh, ros::NodeHandle nh_private)
      : nh_(nh), nh_private_(nh_private) {
    world_frame_name_ = nh_.param<std::string>("world_frame_name", "world");
    body_frame_name_ = nh_.param<std::string>("body_frame_name", "body");
    endeffector_frame_name_ = nh_.param<std::string>("endeffector_frame_name", "tool");
    map_frame_name_ = nh_.param<std::string>("map_frame_name", "map");
    pub_markers_ =
        nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 1);
    pub_trajectory_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        "command/trajectory", 1);

    double v_max = nh_.param<double>("v_max", 1.0);
    double a_max = nh_.param<double>("a_max", 1.0);
    planner_.setLimits(v_max, a_max);

    // get static transforms
    Eigen::Affine3d T_W_M, T_B_E;
    getStaticTransform(world_frame_name_, map_frame_name_, &T_W_M);
    getStaticTransform(body_frame_name_, endeffector_frame_name_, &T_B_E);
    planner_.setStaticFrames(T_B_E, T_W_M);


    ROS_INFO_STREAM("Wait");
    ros::Duration(3.0).sleep();
    ROS_INFO_STREAM("publish");
    test();
  }

  void test(){
    Eigen::Affine3d T_W_B;
    getStaticTransform(world_frame_name_, body_frame_name_, &T_W_B);
    planner_.setDynamicFrames(T_W_B);

    Eigen::Affine3d T_W_B_contact;
    Eigen::Vector3d normal_W, position_W;
    normal_W << 1.0, 0.0, 0.0;
    position_W << 1.0, 1.0, 1.0;
    Eigen::Vector3d force_W = normal_W*5;
    planner_.getT_W_Bcontact(position_W, normal_W, &T_W_B_contact);

    mav_msgs::EigenTrajectoryPoint::Vector trajectory;
    planner_.planTrajectory(T_W_B, T_W_B_contact, &trajectory);
    planner_.planForce(force_W, 5.0, 0.15, &trajectory);
    planner_.planTrajectory(T_W_B_contact, T_W_B, &trajectory);
    visualization_msgs::MarkerArray markers;
    double distance = 0.0; // Distance by which to seperate additional markers.
    // Set 0.0 to disable.

    std::string frame_id = world_frame_name_;
    mav_trajectory_generation::drawMavSampledTrajectory(
        trajectory, distance, frame_id, &markers);

    pub_markers_.publish(markers);

      trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory, &msg);
    msg.header.frame_id = world_frame_name_;
    msg.header.stamp = ros::Time::now();
    pub_trajectory_.publish(msg);

  }

  void getStaticTransform(const std::string& frame_A, const std::string& frame_B,
                          Eigen::Affine3d* T_A_B) {
    if (frame_A == frame_B) {
      *T_A_B = Eigen::Affine3d::Identity();
      return;
    }
    tf::StampedTransform tf_T_A_B;
    try {
      ros::Duration(0.5).sleep();  // wait for transforms to update
      tf_listener_.waitForTransform(frame_A, frame_B, ros::Time(0), ros::Duration(5.0));
      tf_listener_.lookupTransform(frame_A, frame_B, ros::Time(0), tf_T_A_B);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    tf::transformTFToEigen(tf_T_A_B, *T_A_B);

    ROS_INFO_STREAM("Updated static transform " << frame_A << " -> " << frame_B);
  }

 private:
  std::string world_frame_name_;        // W frame
  std::string body_frame_name_;         // B frame
  std::string endeffector_frame_name_;  // E frame
  std::string map_frame_name_;          // M frame

  // Ros stuff
  planning::SurfacePlanner planner_;
  tf::TransformListener tf_listener_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;
};
}  // namespace cad_percept

#endif  // CPT_ROS_INCLUDE_CPT_ROS_NODES_OMAV_TOUCH_PLANNER_H_
