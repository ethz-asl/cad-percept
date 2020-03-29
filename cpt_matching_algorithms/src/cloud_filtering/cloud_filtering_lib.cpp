#include "cloud_filtering/cloud_filtering_lib.h"

void CloudFilterLib::static_object_filter(int structure_threshold,
                                          pcl::PointCloud<pcl::PointXYZ>& lidar_frame,
                                          const pcl::PointCloud<pcl::PointXYZI> static_structure) {
  std::cout << "////  Static Object Filter  ////" << std::endl;

  pcl::ExtractIndices<pcl::PointXYZ> indices_filter;

  if (lidar_frame.size() != static_structure.size()) {
    std::cout << "Error: The sizes of the two point cloud do not match" << std::endl;
    std::cout << "Skip Static Object Fitler" << std::endl;
    return;
  }

  std::vector<int> rm_idx;
  for (int idx = 0; idx < lidar_frame.size(); ++idx) {
    if (static_structure.points[idx].intensity < structure_threshold) {
      rm_idx.push_back(idx);
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_frame_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  *lidar_frame_ptr = lidar_frame;
  boost::shared_ptr<std::vector<int>> inliers_ptr = boost::make_shared<std::vector<int>>(rm_idx);
  indices_filter.setInputCloud(lidar_frame_ptr);
  indices_filter.setIndices(inliers_ptr);
  indices_filter.setNegative(true);
  indices_filter.filter(*lidar_frame_ptr);
  lidar_frame = *lidar_frame_ptr;

  std::cout << "Lidar frame filtered by Static Object Filter with threshold " << structure_threshold
            << std::endl;
  std::cout << "Point Cloud size: " << lidar_frame.size()
            << " Removed: " << static_structure.size() - lidar_frame.size() << std::endl;
}

void CloudFilterLib::voxel_centroid_filter(float search_radius,
                                           pcl::PointCloud<pcl::PointXYZ>& lidar_frame) {
  std::cout << "////    Voxel Centroid Filter   ////" << std::endl;
  int prev_size = lidar_frame.size();

  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_frame_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  *lidar_frame_ptr = lidar_frame;
  pcl::UniformSampling<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(lidar_frame_ptr);
  voxel_filter.setRadiusSearch(search_radius);
  voxel_filter.filter(*lidar_frame_ptr);
  lidar_frame = *lidar_frame_ptr;

  std::cout << "Point Cloud size: " << lidar_frame.size()
            << " Removed: " << prev_size - lidar_frame.size() << std::endl;
}