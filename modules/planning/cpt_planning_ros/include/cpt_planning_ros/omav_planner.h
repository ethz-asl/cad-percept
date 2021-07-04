#ifndef CPT_PLANNING_ROS_OMAV_PLANNER_H
#define CPT_PLANNING_ROS_OMAV_PLANNER_H
#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/coordinates/face_coords.h>
#include <cpt_planning/coordinates/uv_mapping.h>
#include <cpt_planning/interface/mesh_manifold_interface.h>
#include <cpt_ros/mesh_model_publisher.h>
#include <geometry_msgs/PoseArray.h>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <nav_msgs/Odometry.h>
#include <rmpcpp/core/space.h>
#include <rmpcpp/core/state.h>
#include <rmpcpp/eval/trapezoidal_integrator.h>
#include <rmpcpp/policies/simple_target_policy.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/MarkerArray.h>

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
class OMAVPlanner {
  using RMPG = cad_percept::planning::MeshManifoldInterface;
  using LinSpace = rmpcpp::Space<3>;
  using TargetPolicy = rmpcpp::SimpleTargetPolicy<LinSpace>;
  using TargetPolicyPtr = std::shared_ptr<TargetPolicy>;
  using Policies = std::vector<std::shared_ptr<rmpcpp::PolicyBase<LinSpace>>>;
  using PolicyIntegrator = rmpcpp::TrapezoidalIntegrator<Policies, RMPG>;

  enum class FlightMode {
    FreeFlight,         // uses mesh only for safety
    Terrain,            // flies relative to terrain, no orientation control
    TerrainLevelYaw,    // flies relative to terrain, yaw always in flight direction
    TerrainOrientation  // full relative orientation
  };

  enum class OutputMode {
    Trajectory,  // integrate a trajectory
    Velocity     // direct velocity control
  };

 public:
 private:
  void runPlanner();

  void getPolicyFreeFlight(Policies *policies);
  void getPolicyTerrain(Policies *policies);

  void outputVelocities(Policies *policies);
  void outputTrajectory(Policies *policies);

  FlightMode flight_mode_;
  OutputMode output_mode_;

  // RMP & Mesh objects
  cad_percept::cgal::MeshModel::Ptr model_;
  cad_percept::planning::UVMapping *mapping_;
  cad_percept::planning::MeshManifoldInterface *manifold_;

  // ROS subcribers & publishers
  cad_percept::MeshModelPublisher pub_mesh_3d_;
  cad_percept::MeshModelPublisher pub_mesh_2d_;
  ros::Publisher pub_marker_;

  ros::Publisher pub_trajectory_;  // commanded trajectory
  ros::Publisher pub_velocity_;    // commanded velocity

  ros::Subscriber sub_joystick_;  // setpoint via joystick
  ros::Subscriber sub_position_;  // position setpoint
  ros::Subscriber sub_odometry_;  // curent odom
};

}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_ROS_OMAV_PLANNER_H
