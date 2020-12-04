#include "cpt_pointlaser_hal/hal_routine_executer.h"

#include <cpt_pointlaser_msgs/HighAccuracyLocalization.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>

namespace cad_percept {
namespace pointlaser_hal {

HALRoutineExecuter::HALRoutineExecuter(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  // Subscribe to services.
  hal_move_arm_to_initial_pose_client_ =
      nh_.serviceClient<std_srvs::Empty>("/hal_move_arm_to_initial_pose");
  hal_initialize_localization_client_ =
      nh_.serviceClient<std_srvs::Empty>("/hal_initialize_localization");
  hal_visit_poses_client_ = nh_.serviceClient<std_srvs::Empty>("/hal_visit_poses");
  hal_optimize_client_ =
      nh_.serviceClient<cpt_pointlaser_msgs::HighAccuracyLocalization>("/hal_optimize");
  // Advertise topic with corrected base pose.
  corrected_base_pose_in_world_pub_ =
      nh_private_.advertise<geometry_msgs::Pose>("/hal_corrected_base_pose_in_world", 1);
}

}  // namespace pointlaser_hal
}  // namespace cad_percept
