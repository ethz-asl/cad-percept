#include "test_Matcher/test_Matcher.h"

#include <pcl/filters/voxel_grid.h>
#include <ros/package.h>

namespace cad_percept {
namespace matching_algorithms {

void test_Matcher::go_icp_match(float (&transformTR)[6]) {
  // Instructions: load git, then in terminal "cmake", then "make" should create executable GoICP
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "             Go-ICP matcher started            " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "Make sure $(find matching_algorithms)/../../Go-ICP/GoICP.exe exists" << std::endl;

  // Centralize point clouds and scale them to [-1,1]³
  PointCloud go_icp_lidar = lidar_frame;
  PointCloud go_icp_map = sample_map;

  // Downsample lidar
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud = go_icp_lidar;
  float leafsize = nh_private_.param<float>("leafsize", 0.1);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leafsize, leafsize, leafsize);
  sor.filter(*cloud);
  go_icp_lidar = *cloud;

  // Find translation for centralization
  pcl::CentroidPoint<pcl::PointXYZ> centroid_lidar;
  for (PointCloud::iterator i = go_icp_lidar.points.begin(); i < go_icp_lidar.points.end(); i++) {
    centroid_lidar.add(*i);
  }
  pcl::PointXYZ transl_lidar;
  centroid_lidar.get(transl_lidar);

  pcl::CentroidPoint<pcl::PointXYZ> centroid_map;
  for (PointCloud::iterator i = go_icp_map.points.begin(); i < go_icp_map.points.end(); i++) {
    centroid_lidar.add(*i);
  }
  pcl::PointXYZ transl_map;
  centroid_lidar.get(transl_map);

  // Apply transformation
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
  for (PointCloud::iterator i = go_icp_lidar.points.begin(); i < go_icp_lidar.points.end(); i++) {
    if (sqrt(pow(i->x, 2) + pow(i->y, 2) + pow(i->z, 2)) >= max_dist_lidar) {
      max_dist_lidar = sqrt(pow(i->x, 2) + pow(i->y, 2) + pow(i->z, 2));
    }
  }

  float max_dist_map = 0;
  for (PointCloud::iterator i = go_icp_map.points.begin(); i < go_icp_map.points.end(); i++) {
    if (sqrt(pow(i->x, 2) + pow(i->y, 2) + pow(i->z, 2)) >= max_dist_map) {
      max_dist_map = sqrt(pow(i->x, 2) + pow(i->y, 2) + pow(i->z, 2));
    }
  }
  float max_dist = std::max(max_dist_map, max_dist_lidar);

  // Scale point clouds
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
  chdir(ros::package::getPath("matching_algorithms").c_str());
  chdir("../../Go-ICP/");
  std::ofstream map_file("map.txt");  // change cwd to node in launch file
  map_file << go_icp_map.width << std::endl;
  for (PointCloud::iterator i = go_icp_map.points.begin(); i < go_icp_map.points.end(); i++) {
    map_file << i->x << " " << i->y << " " << i->z << std::endl;
  }
  std::cout << "Map.txt created" << std::endl;
  map_file.close();

  std::ofstream lidar_file("lidar_frame.txt");
  lidar_file << go_icp_lidar.width << std::endl;
  for (PointCloud::iterator i = go_icp_lidar.points.begin(); i < go_icp_lidar.points.end(); i++) {
    lidar_file << i->x << " " << i->y << " " << i->z << std::endl;
  }
  std::cout << "lidar_frame.txt created" << std::endl;
  lidar_file.close();

  std::string downsample_points = nh_private_.param<std::string>("downsample", "1000");
  std::cout << "Start Go-ICP" << std::endl;
  std::string command =
      "./GoICP map.txt lidar_frame.txt " + downsample_points + " demo/config.txt output.txt";
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
  Eigen::Matrix4d final_transf = go_icp_trans.inverse();

  Eigen::Matrix3d final_rot = final_transf.block(0, 0, 3, 3);
  Eigen::Vector3f final_euler = final_rot.cast<float>().eulerAngles(0, 1, 2);

  // Revert scaling and translation
  transformTR[0] = final_transf(0, 3) * max_dist - transl_lidar.x + transl_map.x;
  transformTR[1] = final_transf(1, 3) * max_dist - transl_lidar.y + transl_map.y;
  transformTR[2] = final_transf(2, 3) * max_dist - transl_lidar.z + transl_map.z;

  transformTR[3] = final_euler(0);
  transformTR[4] = final_euler(1);
  transformTR[5] = final_euler(2);
}

}  // namespace matching_algorithms
}  // namespace cad_percept
