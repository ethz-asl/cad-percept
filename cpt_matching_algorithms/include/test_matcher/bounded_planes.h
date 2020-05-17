#ifndef BOUNDED_PLANES_H_
#define BOUNDED_PLANES_H_

#include <cgal_conversions/eigen_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <pcl/common/centroid.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <fstream>

namespace cad_percept {
namespace matching_algorithms {

class BoundedPlanes {
 public:
  BoundedPlanes(){};
  BoundedPlanes(const std::vector<pcl::PointCloud<pcl::PointXYZ>> &extracted_map_inliers,
                const std::vector<Eigen::Vector3d> &plane_normals,
                const Eigen::Vector3f &point_in_map);
  // Get point cloud with centroids and normals, get matrix containing room
  // boundaries
  void getPlaneInformations(pcl::PointCloud<pcl::PointNormal> &planes_out,
                            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &boundaries_out);

  // Get plane normals and centroids
  pcl::PointCloud<pcl::PointNormal> getPlaneCentroidsAndNormals();

  // Get info if point projection lies on plane
  bool isProjectionOfPointOnPlane(Eigen::Vector3f point, int map_plane_nr);

  // Get number of planes
  int getPlaneNumber();

  // Print information about each plane
  void dispAllPlanes();

  // Save map plane information to yaml file
  void saveToYamlFile(std::string file_name);

  // Load map plane information from yaml file
  void loadFromYamlFile(std::string file_name);

 private:
  pcl::PointCloud<pcl::PointNormal> plane_centroid_with_normals_;
  Eigen::Matrix<float, 6, Eigen::Dynamic> plane_boundaries_;
};

}  // namespace matching_algorithms
}  // namespace cad_percept

#endif