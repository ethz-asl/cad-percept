#include "plane_extraction/plane_extraction_lib.h"

std::vector<pcl::PointCloud<pcl::PointXYZRGB>> PlaneExtractionLib::pcl_plane_extraction(
    const pcl::PointCloud<pcl::PointXYZ> lidar_frame, int max_number_of_plane,
    int min_number_of_inlier, ros::Publisher &plane_pub_, std::string tf_map_frame) {
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "         PCL Plane Extraction started          " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>> extracted_planes;
  pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_inlier_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_inlier_points(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  // Find plane with PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_lidar(new pcl::PointCloud<pcl::PointXYZ>);
  *plane_lidar = lidar_frame;

  std::cout << "Setup Plane Model Extraction" << std::endl;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);

  std::cout << "Start to extract planes" << std::endl;

  pcl::ExtractIndices<pcl::PointXYZ> indices_filter;

  do {
    seg.setInputCloud(plane_lidar);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() > min_number_of_inlier) {
      for (std::size_t i = 0; i < inliers->indices.size(); ++i)
        extracted_inlier_points->push_back(plane_lidar->points[inliers->indices[i]]);

      pcl::copyPointCloud(*extracted_inlier_points, *colored_inlier_points);
      extracted_planes.push_back(*colored_inlier_points);
      extracted_inlier_points->clear();
      std::cout << "Plane found (nr. " << extracted_planes.size() << ")" << std::endl;

      indices_filter.setInputCloud(plane_lidar);
      indices_filter.setIndices(inliers);
      indices_filter.setNegative(true);
      indices_filter.filter(*plane_lidar);
    }
  } while (extracted_planes.size() < max_number_of_plane &&
           inliers->indices.size() > min_number_of_inlier);

  // Visualize plane
  std::cout << "Start Visualization" << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_point_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  int color[8][3] = {{0, 0, 0},     {255, 0, 0},   {0, 255, 0},   {0, 0, 255},
                     {255, 255, 0}, {255, 0, 255}, {0, 255, 255}, {255, 255, 255}};
  if (extracted_planes.size() == 0) {
    std::cout << "No planes found" << std::endl;
  } else {
    std::cout << "Found " << extracted_planes.size() << " planes, visualize plane inliers... "
              << std::endl;
    for (std::size_t i = 0; i < extracted_planes.size(); ++i) {
      for (std::size_t j = 0; j < extracted_planes[i].points.size(); ++j) {
        extracted_planes[i].points[j].r = color[i % 8][0];
        extracted_planes[i].points[j].b = color[i % 8][1];
        extracted_planes[i].points[j].g = color[i % 8][2];
        extracted_planes[i].points[j].a = 255;
      }
      *segmented_point_cloud += extracted_planes[i];
    }
  }
  sensor_msgs::PointCloud2 segmentation_mesg;
  segmented_point_cloud->header.frame_id = tf_map_frame;
  pcl::toROSMsg(*segmented_point_cloud, segmentation_mesg);
  plane_pub_.publish(segmentation_mesg);
  std::cout << "Publish plane segmentation" << std::endl;

  return extracted_planes;
}
