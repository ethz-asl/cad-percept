#ifndef CPT_PLANNING_ROS_OMAV_PLANNER_H
#define CPT_PLANNING_ROS_OMAV_PLANNER_H
#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/coordinates/face_coords.h>
#include <cpt_planning/coordinates/uv_mapping.h>
#include <cpt_planning/interface/mesh_manifold_interface.h>
#include <cpt_planning_ros/RMPConfigConfig.h>
#include <cpt_ros/mesh_model_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseArray.h>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <nav_msgs/Odometry.h>
#include <rmpcpp/core/space.h>
#include <rmpcpp/core/state.h>
#include <rmpcpp/eval/trapezoidal_integrator.h>
#include <rmpcpp/policies/repulsive_policy.h>
#include <rmpcpp/policies/simple_target_policy.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/MarkerArray.h>

#include <type_traits>

namespace cad_percept {
namespace planning {

/***
 * Planner that supports the following actions:
 * - Freeflight and terrain relative mode incl switch
 *
 * - User Joystick input at any time
 * - go to 3D location via topic
 * - Dynamic reconfigure of policies
 *
 * Outputs:
 *  - Trajectory visualization
 *  - 2D and 3D mesh
 *
 */

// flags for the outputmode
enum class OutputMode {
  Trajectory,  // integrate a trajectory
  Velocity     // direct velocity control
};

enum class FlightMode {
  FreeFlight,         // uses mesh only for safety
  Terrain,            // flies relative to terrain, no orientation control
  TerrainLevelYaw,    // flies relative to terrain, yaw always in flight direction
  TerrainOrientation  // full relative orientation
};

class OMAVPlanner {
  using RMPG = cad_percept::planning::MeshManifoldInterface;
  using LinSpace = rmpcpp::Space<3>;
  using TargetPolicy = rmpcpp::SimpleTargetPolicy<LinSpace>;
  using RepulsivePolicy = rmpcpp::RepulsivePolicy<LinSpace>;
  using TargetPolicyPtr = std::shared_ptr<TargetPolicy>;
  using Policies = std::vector<std::shared_ptr<rmpcpp::PolicyBase<LinSpace>>>;
  using PolicyIntegrator = rmpcpp::TrapezoidalIntegrator<rmpcpp::PolicyBase<LinSpace>, RMPG>;

  struct FixedParams {
    std::string mesh_frame;
    std::string enu_frame;
    std::string odom_frame;
    std::string body_frame;
    std::string current_reference_frame;
    std::string mesh_path;

    Eigen::Vector3d mesh_zero;
    double mesh_zero_angle;
  };

 public:
  OMAVPlanner(ros::NodeHandle nh, ros::NodeHandle nh_private);
  OMAVPlanner() = delete;

 private:
  void runPlanner();

  // one liner helpers
  inline bool allInitialized() const {
    return mesh_loaded_ && frames_received_ && odom_received_ && zeroed_;
  }
  inline Eigen::Vector3d getVelocityENU() { return T_enu_odom_.rotation() * v_odom_body_; }
  inline Eigen::Vector3d getPositionENU() { return (T_enu_odom_ * T_odom_body_).translation(); }

  void getPolicyFreeFlight(Policies *policies);
  void getPolicyTerrain(Policies *policies);

  void generateVelocitiesOdom(const Policies &policies, Eigen::Vector3d *velocity);
  void generateTrajectoryOdom(const Policies &policies,
                              mav_msgs::EigenTrajectoryPoint::Vector *trajectory);

  // config & startup stuff
  void readConfig();
  void loadMesh();

  // callbacks to update stuff
  void joystickCallback(const sensor_msgs::JoyConstPtr &joy);
  void presenterCallback(const geometry_msgs::Vector3StampedPtr& vctr);
  void odometryCallback(const nav_msgs::OdometryConstPtr &odom);
  void tfUpdateCallback(const ros::TimerEvent &event);
  void configCallback(cpt_planning_ros::RMPConfigConfig &config, uint32_t level);

  // publishing method
  void publishTrajectory(const mav_msgs::EigenTrajectoryPoint::Vector &trajectory_odom);

  void publishMarkers();

  FlightMode flight_mode_{FlightMode::Terrain};  // Fix these for now
  OutputMode output_mode_{OutputMode::Trajectory};

  // RMP & Mesh objects
  cad_percept::cgal::MeshModel::Ptr model_enu_;
  cad_percept::planning::UVMapping *mapping_;
  std::shared_ptr<cad_percept::planning::MeshManifoldInterface> manifold_;

  // ROS subcribers & publishers
  cad_percept::MeshModelPublisher pub_mesh_;
  ros::NodeHandle nh_, nh_private_;

  ros::Publisher pub_marker_;
  ros::Publisher pub_trajectory_;  // commanded trajectory
  ros::Publisher pub_velocity_;    // commanded velocity

  ros::Subscriber sub_joystick_;  // setpoint via joystick
  ros::Subscriber sub_odometry_;  // curent odom
  ros::Subscriber sub_presenter_;  // curent odom

  ros::Timer tf_update_timer_;
  // UAV State
  Eigen::Affine3d T_odom_body_, T_enu_odom_, T_enu_mesh_;
  Eigen::Vector3d v_odom_body_;

  tf::TransformListener listener_;
  dynamic_reconfigure::Server<cpt_planning_ros::RMPConfigConfig> server_;

  // desired targets
  Eigen::Vector3d target_temp_uvh_, target_temp_xyz_;
  Eigen::Vector3d target_uvh_, target_xyz_;  // in ENU frame

  // Configuration
  cpt_planning_ros::RMPConfigConfig dynamic_params_;
  FixedParams fixed_params_;

  double dt_{0.01};                    // [s] temporal resolutions for trajectories
  double max_traject_duration_{25.0};  // [s] amount of lookahead for integration

  bool mesh_loaded_{false};
  bool frames_received_{false};
  bool odom_received_{false};
  bool zeroed_{false};
  bool last_button_state_{false};
};

}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_ROS_OMAV_PLANNER_H
