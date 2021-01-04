#include "cpt_pointlaser_hal/hal_routine_executer.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "hal_routine_executer");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::shared_ptr<cad_percept::pointlaser_hal::HALRoutineExecuter> auto_node;

  auto_node = std::make_shared<cad_percept::pointlaser_hal::HALRoutineExecuter>(nh, nh_private);

  ros::spin();
  return 0;
}