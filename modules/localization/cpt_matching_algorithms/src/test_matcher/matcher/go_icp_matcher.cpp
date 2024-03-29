#include "test_matcher/test_matcher.h"

#include <pcl/filters/voxel_grid.h>
#include <ros/package.h>

namespace cad_percept {
namespace matching_algorithms {

void TestMatcher::goicpMatch() {
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "             Go-ICP matcher started            " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;

  std::string downsample_points = nh_private_.param<std::string>("GoICPdownsample", "1000");
  std::string goicp_location = nh_private_.param<std::string>("goicp_folder", "fail");

  PointCloud go_icp_lidar = lidar_scan_;
  PointCloud go_icp_map = sample_map_;

  // Find translation for centralization
  pcl::PointXYZ transl_lidar;
  pcl::computeCentroid(go_icp_lidar, transl_lidar);

  pcl::PointXYZ transl_map;
  pcl::computeCentroid(go_icp_map, transl_map);

  // Centralize point clouds
  Eigen::Matrix4d transform_lidar = Eigen::Matrix4d::Identity();
  Eigen::Vector3d translation_lidar(transl_lidar.x, transl_lidar.y, transl_lidar.z);
  transform_lidar.block(0, 3, 3, 1) = translation_lidar;
  pcl::transformPointCloud(go_icp_lidar, go_icp_lidar, transform_lidar);

  Eigen::Matrix4d transform_map = Eigen::Matrix4d::Identity();
  Eigen::Vector3d translation_map(transl_map.x, transl_map.y, transl_map.z);
  transform_map.block(0, 3, 3, 1) = translation_map;
  pcl::transformPointCloud(go_icp_map, go_icp_map, transform_map);

  // Find point, which is furthest away from centroid for scaling
  float max_dist_lidar = 0;
  for (auto point : go_icp_lidar.points) {
    if (sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2)) >= max_dist_lidar) {
      max_dist_lidar = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
    }
  }

  float max_dist_map = 0;
  for (auto point : go_icp_map.points) {
    if (sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2)) >= max_dist_map) {
      max_dist_lidar = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
    }
  }
  float max_dist = std::max(max_dist_map, max_dist_lidar);

  // Scale point cloud to [-1,1]³
  Eigen::Matrix4d trans_scale_lidar = Eigen::Matrix4d::Identity();
  trans_scale_lidar(0, 0) = trans_scale_lidar(0, 0) / max_dist;
  trans_scale_lidar(1, 1) = trans_scale_lidar(1, 1) / max_dist;
  trans_scale_lidar(2, 2) = trans_scale_lidar(2, 2) / max_dist;
  pcl::transformPointCloud(go_icp_lidar, go_icp_lidar, trans_scale_lidar);

  Eigen::Matrix4d trans_scale_map = Eigen::Matrix4d::Identity();
  trans_scale_map(0, 0) = trans_scale_map(0, 0) / max_dist;
  trans_scale_map(1, 1) = trans_scale_map(1, 1) / max_dist;
  trans_scale_map(2, 2) = trans_scale_map(2, 2) / max_dist;
  pcl::transformPointCloud(go_icp_map, go_icp_map, trans_scale_map);

  // Create txt files of point clouds, required for Go-ICP
  chdir(goicp_location.c_str());
  std::ofstream map_file("map.txt");
  map_file << go_icp_map.width << std::endl;
  for (PointCloud::iterator i = go_icp_map.points.begin(); i < go_icp_map.points.end(); i++) {
    map_file << i->x << " " << i->y << " " << i->z << std::endl;
  }
  std::cout << "Map.txt created" << std::endl;
  map_file.close();

  std::ofstream lidar_file("lidar_scan.txt");
  lidar_file << go_icp_lidar.width << std::endl;
  for (PointCloud::iterator i = go_icp_lidar.points.begin(); i < go_icp_lidar.points.end(); i++) {
    lidar_file << i->x << " " << i->y << " " << i->z << std::endl;
  }
  std::cout << "lidar_scan.txt created" << std::endl;
  lidar_file.close();

  std::cout << "Start Go-ICP" << std::endl;
  std::string command =
      "./GoICP map.txt lidar_scan.txt " + downsample_points + " config.txt output.txt";
  system(command.c_str());
  std::cout << "Go-ICP finished" << std::endl;

  // Read results
  float time_needed;
  Eigen::Matrix4d go_icp_trans = Eigen::Matrix4d::Identity();

  std::ifstream output("output.txt");
  if (output.is_open()) {
    std::vector<float> values;
    std::copy(std::istream_iterator<float>(output), std::istream_iterator<float>(),
              std::back_inserter(values));

    time_needed = values[0];
    go_icp_trans(0, 0) = values[1];
    go_icp_trans(0, 1) = values[2];
    go_icp_trans(0, 2) = values[3];
    go_icp_trans(1, 0) = values[4];
    go_icp_trans(1, 1) = values[5];
    go_icp_trans(1, 2) = values[6];
    go_icp_trans(2, 0) = values[7];
    go_icp_trans(2, 1) = values[8];
    go_icp_trans(2, 2) = values[9];
    go_icp_trans(0, 3) = values[10];
    go_icp_trans(1, 3) = values[11];
    go_icp_trans(2, 3) = values[12];

    std::cout << "Go-ICP needed " << time_needed << " seconds." << std::endl;
  } else {
    std::cout << "Could not read output file" << std::endl;
  }

  // Get matrix of unscaled matrix
  Eigen::Matrix4d final_transf = go_icp_trans;

  Eigen::Matrix3d final_rot = final_transf.block(0, 0, 3, 3);
  Eigen::Quaterniond final_q(final_rot);

  // Revert scaling and translation
  transform_TR_[0] = final_transf(0, 3) * max_dist - transl_lidar.x + transl_map.x;
  transform_TR_[1] = final_transf(1, 3) * max_dist - transl_lidar.y + transl_map.y;
  transform_TR_[2] = final_transf(2, 3) * max_dist - transl_lidar.z + transl_map.z;

  transform_TR_[3] = final_q.w();
  transform_TR_[4] = final_q.x();
  transform_TR_[5] = final_q.y();
  transform_TR_[6] = final_q.z();
}

}  // namespace matching_algorithms
}  // namespace cad_percept
