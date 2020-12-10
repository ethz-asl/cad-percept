#ifndef CPT_POINTLASER_HAL_HAL_ROUTINE_EXECUTER_H_
#define CPT_POINTLASER_HAL_HAL_ROUTINE_EXECUTER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace cad_percept {
namespace pointlaser_hal {

/// \brief Class that assists the user in executing the HAL routine.
class HALRoutineExecuter {
 public:
  HALRoutineExecuter(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

 private:
  ///
  /// \brief Main function, guides the user through the HAL routine.
  void assistUserThroughRoutine();

  // Node handles.
  ros::NodeHandle nh_, nh_private_;
  // Transform broadcaster. (Used to publish the corrected base pose in the world frame, returned by
  // the HAL optimization.
  tf::TransformBroadcaster tf_broadcaster_;
  // Transform listener.
  tf::TransformListener transform_listener_;
  // Service clients and server.
  ros::ServiceClient hal_move_arm_to_initial_pose_client_, hal_initialize_localization_client_,
      hal_visit_poses_client_, hal_optimize_client_;
  // Internal parameters.
  std::string optimized_ee_frame_name_;
};
}  // namespace pointlaser_hal
}  // namespace cad_percept
#endif
