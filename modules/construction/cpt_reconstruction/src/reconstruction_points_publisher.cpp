#include <cpt_reconstruction/reconstruction_points_publisher.h>

namespace cad_percept {
namespace cpt_reconstruction {

ReconstructionPointsPublisher::ReconstructionPointsPublisher(
    ros::NodeHandle nodeHandle) {
  nodeHandle_ = nodeHandle;

  nodeHandle.getParam("SensorType", SENSOR_TYPE_);
  nodeHandle.getParam("ScanPointsFile", SCAN_PATH_FILE_);

  publisher_ =
      nodeHandle_.advertise<sensor_msgs::PointCloud2>("corrected_scan", 1);

  this->publishPoints();

  ros::spin();
}

void ReconstructionPointsPublisher::publishPoints() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scan(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (SENSOR_TYPE_ == 1) {
    pcl::PLYReader reader;
    reader.read(SCAN_PATH_FILE_, *cloud_scan);

    sensor_msgs::PointCloud2 message_data;
    pcl::PCLPointCloud2 pcl2;
    pcl::toPCLPointCloud2(*cloud_scan, pcl2);
    pcl_conversions::moveFromPCL(pcl2, message_data);

    ROS_INFO("Waiting ...");
    sleep(10);
    ROS_INFO("Waiting Done ...");
    publisher_.publish(message_data);
    ROS_INFO("Published_Message");
  }
}

}  // namespace cpt_reconstruction
}  // namespace cad_percept