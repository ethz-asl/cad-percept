#include <ros/ros.h>
#include <thread>

#include "cad_interface/cad_interface.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "cad_interface_node");
  ros::NodeHandle node_handle("~");

  cad_percept::cad_interface::CadInterface cad_interface(node_handle);
  std::string filename;
  node_handle.getParam("/cad_file_pcd", filename);
  cad_interface.load(filename);
  std::thread publish_point_cloud_thread
      (&cad_percept::cad_interface::CadInterface::publishPointCloudThread, &cad_interface);

  try {
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;
  }
  catch (...) {
    ROS_ERROR_STREAM("Unknown Exception");
    return 1;
  }

  publish_point_cloud_thread.join();

  return 0;
}
