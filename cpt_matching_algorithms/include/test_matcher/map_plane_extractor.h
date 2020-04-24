#ifndef MAP_PLANES_H_
#define MAP_PLANES_H_

#include <pcl/common/centroid.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <fstream>

namespace cad_percept {
namespace matching_algorithms {

class MapPlanes {
 public:
  MapPlanes(){};
  MapPlanes(std::vector<pcl::PointCloud<pcl::PointXYZ>> extracted_map_inliers,
            std::vector<Eigen::Vector3d> plane_normals, Eigen::Vector3f point_in_map);
  // Get point cloud with centroids and normals, get matrix containing room
  // boundaries
  void getMapPlaneInformations(
      pcl::PointCloud<pcl::PointNormal> &output_planes,
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &output_boundaries);

  // Get plane normals and centroids
  pcl::PointCloud<pcl::PointNormal> getPlaneCentroidsAndNormals();

  // Get info if point projection lies on plane
  bool isProjectionOfPointOnPlane(Eigen::Vector3f point, int map_plane_nr);

  // Get number of planes of map
  int getMapPlaneNumber();

  // Print information about each plane
  void dispAllPlanes();

  // Save map plane information to yaml file
  void saveToYamlFile(std::string file_name);

  // Load map plane information from yaml file
  void loadFromYamlFile(std::string file_name);

 private:
  pcl::PointCloud<pcl::PointNormal> plane_centroid_with_normals_;
  Eigen::Matrix<float, 6, Eigen::Dynamic> plane_boundaries;
};

}  // namespace matching_algorithms
}  // namespace cad_percept

#endif