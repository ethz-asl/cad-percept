#include <cpt_reconstruction/reconstruction_points_publisher.h>

// ROS
#include "cpt_reconstruction/coordinates.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

// STD
#include <iostream>
#include <string>

// PCL
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace cad_percept {
namespace cpt_reconstruction {

ReconstructionPointsPublisher::ReconstructionPointsPublisher(ros::NodeHandle nodeHandle,
                                                             std::string filename, int batch_size) {
  nodeHandle_ = nodeHandle;
  filename_ = filename;
  batch_size = batch_size;
}

void ReconstructionPointsPublisher::publishPoints() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scan(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PLYReader reader;
  reader.read(filename_, *cloud_scan);

  ros::Publisher chatter_publisher =
      nodeHandle_.advertise<::cpt_reconstruction::coordinates>("point", 1000);
  ros::Rate loop_rate(0.5);

  int count = 0;
  while (ros::ok() && count < cloud_scan->size()) {
    pcl::PointXYZ p = (*cloud_scan)[count];

    ::cpt_reconstruction::coordinates msg;
    msg.x = p.x;
    msg.y = p.y;
    msg.z = p.z;

    ROS_INFO("[Talker] I published %f %f %f\n", msg.x, msg.y, msg.z);

    chatter_publisher.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }
}

}  // namespace cpt_reconstruction
}  // namespace cad_percept