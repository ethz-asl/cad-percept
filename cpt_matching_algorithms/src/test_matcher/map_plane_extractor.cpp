#include "test_matcher/map_plane_extractor.h"

namespace cad_percept {
namespace matching_algorithms {

MapPlanes::MapPlanes(std::vector<pcl::PointCloud<pcl::PointXYZ>> extracted_map_inliers,
                     std::vector<Eigen::Vector3d> plane_normals, Eigen::Vector3f point_in_map) {
  // Create cloud with centroids and normals
  pcl::PointNormal norm_point;
  pcl::PointXYZ plane_centroid;
  int i = 0;
  for (auto plane_inlier : extracted_map_inliers) {
    pcl::computeCentroid(plane_inlier, plane_centroid);
    norm_point.x = plane_centroid.x;
    norm_point.y = plane_centroid.y;
    norm_point.z = plane_centroid.z;

    norm_point.normal_x = plane_normals[i][0];
    norm_point.normal_y = plane_normals[i][1];
    norm_point.normal_z = plane_normals[i][2];

    // Set main part of normals to convention
    if ((Eigen::Vector3f(norm_point.x, norm_point.y, norm_point.z) - point_in_map)
            .dot(Eigen::Vector3f(norm_point.normal_x, norm_point.normal_y, norm_point.normal_z)) <
        0) {
      norm_point.normal_x = -norm_point.normal_x;
      norm_point.normal_y = -norm_point.normal_y;
      norm_point.normal_z = -norm_point.normal_z;
    }
    plane_centroid_with_normals_.push_back(norm_point);
    i++;
  }
  std::cout << "Loaded normals and centroids of planes, find boundaries of planes ..." << std::endl;
  // Find boundaries of plane (assume rectangular walls, only works if planes
  // have only normals in x, y or z yet)
  plane_boundaries_ = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(
      6, plane_centroid_with_normals_.size());
  std::vector<float> dim_x_of_plane;
  std::vector<float> dim_y_of_plane;
  std::vector<float> dim_z_of_plane;

  i = 0;
  for (auto plane_inlier : extracted_map_inliers) {
    dim_x_of_plane.clear();
    dim_y_of_plane.clear();
    dim_z_of_plane.clear();
    for (auto inlier_point : plane_inlier) {
      dim_x_of_plane.push_back(inlier_point.x);
      dim_y_of_plane.push_back(inlier_point.y);
      dim_z_of_plane.push_back(inlier_point.z);
    }
    plane_boundaries_(0, i) = *std::min_element(dim_x_of_plane.begin(),
                                                dim_x_of_plane.end());  // x_min
    plane_boundaries_(1, i) = *std::max_element(dim_x_of_plane.begin(),
                                                dim_x_of_plane.end());  // x_max
    plane_boundaries_(2, i) = *std::min_element(dim_y_of_plane.begin(),
                                                dim_y_of_plane.end());  // y_min
    plane_boundaries_(3, i) = *std::max_element(dim_y_of_plane.begin(),
                                                dim_y_of_plane.end());  // y_max
    plane_boundaries_(4, i) = *std::min_element(dim_z_of_plane.begin(),
                                                dim_z_of_plane.end());  // z_min
    plane_boundaries_(5, i) = *std::max_element(dim_z_of_plane.begin(),
                                                dim_z_of_plane.end());  // z_max

    i++;
  }
};

bool MapPlanes::isProjectionOfPointOnPlane(Eigen::Vector3f point, int map_plane_nr) {
  pcl::PointNormal plane = plane_centroid_with_normals_.points[map_plane_nr];
  float tol = 0.5;
  if (std::abs(plane.normal_x) > 0.8) {
    return plane_boundaries_(2, map_plane_nr) - tol < point[1] &&
           point[1] < plane_boundaries_(3, map_plane_nr) + tol &&
           plane_boundaries_(4, map_plane_nr) - tol < point[2] &&
           point[2] < plane_boundaries_(5, map_plane_nr) + tol;
  } else if (std::abs(plane.normal_y) > 0.8) {
    return plane_boundaries_(0, map_plane_nr) - tol < point[0] &&
           point[0] < plane_boundaries_(1, map_plane_nr) + tol &&
           plane_boundaries_(4, map_plane_nr) - tol < point[2] &&
           point[2] < plane_boundaries_(5, map_plane_nr) + tol;
  } else if (std::abs(plane.normal_z) > 0.8) {
    return plane_boundaries_(0, map_plane_nr) - tol < point[0] &&
           point[0] < plane_boundaries_(1, map_plane_nr) + tol &&
           plane_boundaries_(2, map_plane_nr) - tol < point[1] &&
           point[1] < plane_boundaries_(3, map_plane_nr) + tol;
  }
  return false;
};

void MapPlanes::getMapPlaneInformations(
    pcl::PointCloud<pcl::PointNormal> &output_planes,
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &output_boundaries) {
  output_planes.clear();
  for (auto plane : plane_centroid_with_normals_) output_planes.push_back(plane);
  output_boundaries = plane_boundaries_;
};

pcl::PointCloud<pcl::PointNormal> MapPlanes::getPlaneCentroidsAndNormals() {
  return plane_centroid_with_normals_;
}

int MapPlanes::getMapPlaneNumber() { return plane_centroid_with_normals_.size(); };

void MapPlanes::dispAllPlanes() {
  int plane_nr = 0;
  std::cout << "Print all information about planes ..." << std::endl;
  for (auto plane : plane_centroid_with_normals_) {
    std::cout << "plane nr. " << plane_nr << " x: " << plane.x << " y: " << plane.y
              << "z : " << plane.z << std::endl;
    std::cout << "normal x: " << plane.normal_x << " normal y: " << plane.normal_y
              << " normal z: " << plane.normal_z << std::endl;
    std::cout << "boundaries: xmin: " << plane_boundaries_(0, plane_nr)
              << " boundaries: xmax: " << plane_boundaries_(1, plane_nr) << std::endl;
    std::cout << "boundaries: ymin: " << plane_boundaries_(2, plane_nr)
              << " boundaries: ymax: " << plane_boundaries_(3, plane_nr) << std::endl;
    std::cout << "boundaries: zmin: " << plane_boundaries_(4, plane_nr)
              << " boundaries: zmax: " << plane_boundaries_(5, plane_nr) << std::endl;
    plane_nr++;
  }
};

void MapPlanes::saveToYamlFile(std::string file_name) {
  // Save data in yaml file
  std::string plane_name = "map_plane_nr_";
  std::string actual_plane_name;
  YAML::Node node;
  node["size"] = plane_centroid_with_normals_.size();
  for (int map_plane_nr = 0; map_plane_nr < plane_centroid_with_normals_.size(); ++map_plane_nr) {
    actual_plane_name = plane_name + std::to_string(map_plane_nr);
    node[actual_plane_name].push_back(plane_centroid_with_normals_.points[map_plane_nr].x);
    node[actual_plane_name].push_back(plane_centroid_with_normals_.points[map_plane_nr].y);
    node[actual_plane_name].push_back(plane_centroid_with_normals_.points[map_plane_nr].z);
    node[actual_plane_name].push_back(plane_centroid_with_normals_.points[map_plane_nr].normal_x);
    node[actual_plane_name].push_back(plane_centroid_with_normals_.points[map_plane_nr].normal_y);
    node[actual_plane_name].push_back(plane_centroid_with_normals_.points[map_plane_nr].normal_z);
    node[actual_plane_name].push_back(plane_boundaries_(0, map_plane_nr));
    node[actual_plane_name].push_back(plane_boundaries_(1, map_plane_nr));
    node[actual_plane_name].push_back(plane_boundaries_(2, map_plane_nr));
    node[actual_plane_name].push_back(plane_boundaries_(3, map_plane_nr));
    node[actual_plane_name].push_back(plane_boundaries_(4, map_plane_nr));
    node[actual_plane_name].push_back(plane_boundaries_(5, map_plane_nr));
  }
  std::ofstream fout(file_name);
  fout << node;
};

void MapPlanes::loadFromYamlFile(std::string file_name) {
  YAML::Node node = YAML::LoadFile(file_name);
  int map_size = node["size"].as<int>();
  plane_boundaries_ = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(6, map_size);
  std::string actual_plane_name;
  std::string plane_name = "map_plane_nr_";
  pcl::PointNormal map_plane;
  for (int map_plane_nr = 0; map_plane_nr < map_size; ++map_plane_nr) {
    actual_plane_name = plane_name + std::to_string(map_plane_nr);
    map_plane.x = node[actual_plane_name][0].as<float>();
    map_plane.y = node[actual_plane_name][1].as<float>();
    map_plane.z = node[actual_plane_name][2].as<float>();
    map_plane.normal_x = node[actual_plane_name][3].as<float>();
    map_plane.normal_y = node[actual_plane_name][4].as<float>();
    map_plane.normal_z = node[actual_plane_name][5].as<float>();
    plane_centroid_with_normals_.push_back(map_plane);
    plane_boundaries_(0, map_plane_nr) = node[actual_plane_name][6].as<float>();
    plane_boundaries_(1, map_plane_nr) = node[actual_plane_name][7].as<float>();
    plane_boundaries_(2, map_plane_nr) = node[actual_plane_name][8].as<float>();
    plane_boundaries_(3, map_plane_nr) = node[actual_plane_name][9].as<float>();
    plane_boundaries_(4, map_plane_nr) = node[actual_plane_name][10].as<float>();
    plane_boundaries_(5, map_plane_nr) = node[actual_plane_name][11].as<float>();
  }
};
}  // namespace matching_algorithms
}  // namespace cad_percept