
#ifndef CPT_ROS_INCLUDE_CPT_ROS_NODES_OMAV_TOUCH_PLANNER_H_
#define CPT_ROS_INCLUDE_CPT_ROS_NODES_OMAV_TOUCH_PLANNER_H_

#include <cpt_planning/coordinates/face_coords.h>
#include <cpt_planning/omav/surface_planner.h>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <random>

namespace cad_percept {

class OmavTouchPlanner {
 public:
  OmavTouchPlanner(ros::NodeHandle nh, ros::NodeHandle nh_private)
      : nh_(nh), nh_private_(nh_private) {
    world_frame_name_ = nh_private_.param<std::string>("world_frame_name", "world");
    body_frame_name_ = nh_private_.param<std::string>("body_frame_name", "body");
    endeffector_frame_name_ = nh_private_.param<std::string>("endeffector_frame_name", "tool");
    map_frame_name_ = nh_private_.param<std::string>("map_frame_name", "map");
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 1);
    pub_trajectory_ =
        nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1);

    double v_max = nh_private_.param<double>("v_max", 1.0);
    double a_max = nh_private_.param<double>("a_max", 1.0);
    planner_.setLimits(v_max, a_max);

    // get static transforms
    Eigen::Affine3d T_W_M, T_B_E;
    getStaticTransform(world_frame_name_, map_frame_name_, &T_W_M);
    getStaticTransform(body_frame_name_, endeffector_frame_name_, &T_B_E);
    planner_.setStaticFrames(T_B_E, T_W_M);

    // Set up mesh
    ROS_INFO_STREAM("load mesh");
    cgal::MeshModel::Ptr model;
    std::string path =
        "/home/mpantic/workspace/main_ws/src/mav_tools/mav_description/meshes/wall_sane.off";
    cgal::MeshModel::create(path, &model);
    planner_.setMesh(model);

    std::vector<double> touch_position =
        nh_private_.param<std::vector<double>>("touch_position", {0.0, 0.0, 0.0});

    nominal_touch_position_ << touch_position[0], touch_position[1],
        touch_position[2];  // 1.48606, 0.77216, 0.668549;

    std::vector<double> touch_random =
        nh_private_.param<std::vector<double>>("touch_random_bounds", {0.0, 0.0, 0.0});
    touch_random_ << touch_random[0], touch_random[1],
        touch_random[2];

    ROS_INFO_STREAM("Touch position: " << nominal_touch_position_);
    ROS_INFO_STREAM("Touch random bounds: " << touch_random_);
    touch_service_ = nh_.advertiseService("touch", &OmavTouchPlanner::touch, this);
    ROS_INFO_STREAM("Wait");
    ros::Duration(3.0).sleep();
    ROS_INFO_STREAM("publish");
  }

  bool touch(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    Eigen::Affine3d T_W_B;
    getStaticTransform(world_frame_name_, body_frame_name_, &T_W_B);
    planner_.setDynamicFrames(T_W_B);

    // plan position
    Eigen::Vector3d pos_map(nominal_touch_position_);

    // add a bit randomness in x and y!
    std::random_device rd;   // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis_x(-touch_random_.x(), touch_random_.x());
    std::uniform_real_distribution<> dis_y(-touch_random_.y(), touch_random_.y());
    std::uniform_real_distribution<> dis_z(-touch_random_.z(), touch_random_.z());

    if (touch_random_.x() > 0.0) {
      pos_map.x() += dis_x(gen);
    }
    if (touch_random_.y() > 0.0) {
      pos_map.y() += dis_y(gen);
    }
    if (touch_random_.z() > 0.0) {
      pos_map.z() += dis_z(gen);
    }
    mav_msgs::EigenTrajectoryPoint::Vector trajectory;
    planner_.planFullContact(pos_map, 2.0, &trajectory);

    // send markers
    visualization_msgs::MarkerArray markers;
    std::string frame_id = world_frame_name_;
    mav_trajectory_generation::drawMavSampledTrajectory(trajectory, 0.0, frame_id, &markers);
    pub_markers_.publish(markers);

    // send trajectory
    trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory, &msg);
    msg.header.frame_id = world_frame_name_;
    msg.header.stamp = ros::Time::now();
    pub_trajectory_.publish(msg);
    return true;
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
  Eigen::Vector3d nominal_touch_position_;
  Eigen::Vector3d touch_random_;

  std::string world_frame_name_;        // W frame
  std::string body_frame_name_;         // B frame
  std::string endeffector_frame_name_;  // E frame
  std::string map_frame_name_;          // M frame

  // Ros stuff
  planning::SurfacePlanner planner_;
  tf::TransformListener tf_listener_;

  ros::ServiceServer touch_service_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;
};
}  // namespace cad_percept

#endif  // CPT_ROS_INCLUDE_CPT_ROS_NODES_OMAV_TOUCH_PLANNER_H_
