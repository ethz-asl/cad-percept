
#ifndef CPT_ROS_INCLUDE_CPT_ROS_NODES_OMAV_TOUCH_PLANNER_H_
#define CPT_ROS_INCLUDE_CPT_ROS_NODES_OMAV_TOUCH_PLANNER_H_

#include <cgal_conversions/mesh_conversions.h>
#include <cgal_msgs/TriangleMeshStamped.h>
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
    selection_frame_name_ = nh_private_.param<std::string>("selection_frame_name", "map_select");
    body_frame_name_ = nh_private_.param<std::string>("body_frame_name", "body");
    endeffector_frame_name_ = nh_private_.param<std::string>("endeffector_frame_name", "tool");
    current_ref_frame_name_ = nh_private_.param<std::string>("current_ref_frame_name", "ouzel/current_reference");

    map_frame_name_ = nh_private_.param<std::string>("map_frame_name", "map");
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 1);
    pub_trajectory_ =
        nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1);

    double v_max = nh_private_.param<double>("v_max", 1.0);
    double a_max = nh_private_.param<double>("a_max", 1.0);
    double dist_ = nh_private_.param<double>("dist", 0.3);
    force_ = nh_private_.param<double>("contact_force", 10.0);
    planner_.setLimits(v_max, a_max);

    // get static transforms
    updateStaticFrames();
    // Set up mesh
    ROS_INFO_STREAM("load mesh");
    cgal::MeshModel::Ptr model;
    std::string path = nh_private_.param<std::string>("mesh_path", "");
    if (!cgal::MeshModel::create(path, &model)) {
      ROS_ERROR_STREAM("INVALID MESH. EXITING");
      exit(1);
    }
    planner_.setMesh(model);

    std::vector<double> touch_position =
        nh_private_.param<std::vector<double>>("touch_position", {0.0, 0.0, 0.0});

    nominal_touch_position_ << touch_position[0], touch_position[1],
        touch_position[2];  // 1.48606, 0.77216, 0.668549;

    std::vector<double> touch_random =
        nh_private_.param<std::vector<double>>("touch_random_bounds", {0.0, 0.0, 0.0});
    touch_random_ << touch_random[0], touch_random[1], touch_random[2];

    pub_mesh_ = nh_.advertise<cgal_msgs::TriangleMeshStamped>("mesh", 1, true);
    cgal_msgs::TriangleMeshStamped msg;
    msg.header.frame_id = map_frame_name_;
    cgal::Polyhedron mesh_poly(model->getMesh());
    cgal::triangleMeshToMsg(mesh_poly, &msg.mesh);
    pub_mesh_.publish(msg);

    ROS_INFO_STREAM("Touch position: " << nominal_touch_position_);
    ROS_INFO_STREAM("Touch random bounds: " << touch_random_);
    touch_service_ = nh_.advertiseService("touch", &OmavTouchPlanner::touch, this);
    untouch_service_ = nh_.advertiseService("untouch", &OmavTouchPlanner::untouch, this);
  }

  bool touch(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    if(update_static_){
      updateStaticFrames();
    }
    Eigen::Affine3d T_W_B;
    if(plan_from_ref_){
      getStaticTransform(world_frame_name_, current_ref_frame_name_, &T_W_B);
    }else{
    getStaticTransform(world_frame_name_, body_frame_name_, &T_W_B);
    }
    planner_.setDynamicFrames(T_W_B);

    mav_msgs::EigenTrajectoryPoint::Vector trajectory;
    Eigen::Affine3d contact_pos_w;
    ROS_INFO_STREAM("Force = " << force_);
    planner_.planFullContact(current_pos_, force_, &trajectory, &contact_pos_w);

    // send markers
    visualization_msgs::MarkerArray markers;
    std::string frame_id = world_frame_name_;
    mav_trajectory_generation::drawMavSampledTrajectorybyTime(trajectory, 0.1, frame_id, &markers);
    pub_markers_.publish(markers);

    // send tf for touch position
    tf::StampedTransform tf_W_touch;
    tf::transformEigenToTF(contact_pos_w, tf_W_touch);
    tf_W_touch.frame_id_ = world_frame_name_;
    tf_W_touch.child_frame_id_ = endeffector_frame_name_ + "_touch";
    tf_broadcast_.sendTransform(tf_W_touch);

    // send trajectory
    trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory, &msg);
    msg.header.frame_id = world_frame_name_;
    msg.header.stamp = ros::Time::now();
    pub_trajectory_.publish(msg);
    return true;
  }

  bool untouch(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    Eigen::Affine3d T_W_B;
    if(update_static_){
      updateStaticFrames();
    }

    if(plan_from_ref_){
      getStaticTransform(world_frame_name_, current_ref_frame_name_, &T_W_B);
    }else{
      getStaticTransform(world_frame_name_, body_frame_name_, &T_W_B);
    }
    planner_.setDynamicFrames(T_W_B);

    mav_msgs::EigenTrajectoryPoint::Vector trajectory;
    Eigen::Affine3d contact_pos_w;
    ROS_INFO_STREAM("Force = " << force_);
    planner_.planFullContactRetract(current_pos_, force_, &trajectory, &contact_pos_w);

    // send markers
    visualization_msgs::MarkerArray markers;
    std::string frame_id = world_frame_name_;
    mav_trajectory_generation::drawMavSampledTrajectorybyTime(trajectory, 0.1, frame_id, &markers);
    pub_markers_.publish(markers);

    // send trajectory
    trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory, &msg);
    msg.header.frame_id = world_frame_name_;
    msg.header.stamp = ros::Time::now();
    pub_trajectory_.publish(msg);

    if(point_ % 3 == 2 ){
      direction_ = !direction_;
      current_pos_.x() += dist_;
    }else{
      if(direction_) // true = +z
      {
        current_pos_.z() += dist_;
      }else{
        current_pos_.z() -= dist_;
      }
    }
    ROS_INFO_STREAM("Current pt = " << current_pos_);
    point_++;

    return true;
  }

  void updateStaticFrames(){
    // get static transforms
    Eigen::Affine3d T_W_M, T_B_E, T_M_S;
    getStaticTransform(world_frame_name_, map_frame_name_, &T_W_M);
    getStaticTransform(body_frame_name_, endeffector_frame_name_, &T_B_E);
    getStaticTransform(map_frame_name_, selection_frame_name_, &T_M_S);

    planner_.setStaticFrames(T_B_E, T_W_M, T_M_S);
  }

  void getStaticTransform(const std::string &frame_A, const std::string &frame_B,
                          Eigen::Affine3d *T_A_B) {
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
  bool update_static_{true};
  bool plan_from_ref_{true};
  double dist_{0.2};
  bool direction_{false}; // false = -z, true = +z
  Eigen::Vector3d current_pos_ {Eigen::Vector3d::Zero()};
  int point_{0};
  double force_;
  Eigen::Vector3d nominal_touch_position_;
  Eigen::Vector3d touch_random_;

  std::string current_ref_frame_name_;  // R_frame
  std::string selection_frame_name_;    // S frame
  std::string world_frame_name_;        // W frame
  std::string body_frame_name_;         // B frame
  std::string endeffector_frame_name_;  // E frame
  std::string map_frame_name_;          // M frame

  // Ros stuff
  planning::SurfacePlanner planner_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcast_;

  ros::ServiceServer touch_service_;
  ros::ServiceServer untouch_service_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;
  ros::Publisher pub_mesh_;
  ros::Publisher pub_evaluate_;
};
}  // namespace cad_percept

#endif  // CPT_ROS_INCLUDE_CPT_ROS_NODES_OMAV_TOUCH_PLANNER_H_
