#include "test_matcher/bounded_planes.h"

namespace cad_percept {
namespace matching_algorithms {

BoundedPlanes::BoundedPlanes(cad_percept::cgal::MeshModel &reference_mesh,
                             std::string cache_folder) {
  // Check if there exists a yaml-file to the mesh
  std::string cache_file_name =
      cache_folder + "map_test_" + std::to_string(reference_mesh.size()) + ".yaml";
  std::ifstream cache_file(cache_file_name);
  if (cache_file.fail()) {
    std::cout << "chached failed" << std::endl;
    // If there is no cached data of the mesh, create a new one
    // Extract planes
    std::unordered_map<std::string, std::string> facetToPlane;
    std::unordered_multimap<std::string, std::string> planeToFacets;

    // Use findAllCoplanarFacets for "coarse" plane extraction
    reference_mesh.findAllCoplanarFacets(&facetToPlane, &planeToFacets, 1.0);

    std::multimap<std::string, std::string> grouped_planeToFacets(planeToFacets.begin(),
                                                                  planeToFacets.end());
    std::string actual_plane_id = "-1";
    cgal::Plane actual_plane;
    cgal::Triangle actual_facet;
    Eigen::Vector3d normal_of_plane;
    pcl::PointNormal plane;
    std::vector<std::vector<float>> inlier(3, std::vector<float>(0));

    // Extract planes with inlier from the coarse plane extraction
    for (auto facet_of_plane : grouped_planeToFacets) {
      actual_facet = reference_mesh.getTriangle(facet_of_plane.second);
      actual_plane = actual_facet.supporting_plane();
      if (facet_of_plane.first.compare(actual_plane_id) != 0) {
        // New plane
        if (actual_plane_id.compare("-1")) {
          // Find centroid and boundaries of previous plane
          plane.x = std::accumulate(inlier[0].begin(), inlier[0].end(), 0.0) / inlier[0].size();
          plane.y = std::accumulate(inlier[1].begin(), inlier[1].end(), 0.0) / inlier[1].size();
          plane.z = std::accumulate(inlier[2].begin(), inlier[2].end(), 0.0) / inlier[2].size();

          plane_boundaries_[0].push_back(
              *std::min_element(inlier[0].begin(), inlier[0].end()));  // x_min
          plane_boundaries_[1].push_back(
              *std::max_element(inlier[0].begin(), inlier[0].end()));  // x_max
          plane_boundaries_[2].push_back(
              *std::min_element(inlier[1].begin(), inlier[1].end()));  // y_min
          plane_boundaries_[3].push_back(
              *std::max_element(inlier[1].begin(), inlier[1].end()));  // y_max
          plane_boundaries_[4].push_back(
              *std::min_element(inlier[2].begin(), inlier[2].end()));  // z_min
          plane_boundaries_[5].push_back(
              *std::max_element(inlier[2].begin(), inlier[2].end()));  // z_max

          plane_centroid_with_normals_.push_back(plane);
        }
        actual_plane_id = facet_of_plane.first;
        // Assume that normal of first facet is normal of the plane
        normal_of_plane =
            cgal::cgalVectorToEigenVector(actual_plane.orthogonal_vector()).normalized();

        plane.normal_x = normal_of_plane(0);
        plane.normal_y = normal_of_plane(1);
        plane.normal_z = normal_of_plane(2);

        inlier[0].clear();
        inlier[1].clear();
        inlier[2].clear();
      }
      // Add inlier
      for (int i = 0; i < 3; i++) {
        inlier[0].push_back(actual_facet.vertex(i).x());
        inlier[1].push_back(actual_facet.vertex(i).y());
        inlier[2].push_back(actual_facet.vertex(i).z());
      }
    }
    // Save as new cache file
    saveToYamlFile(cache_file_name);
  } else {
    // If cached data exist, load data
    std::cout << "Load data from cached map" << std::endl;
    loadFromYamlFile(cache_file_name);
  }
}

bool BoundedPlanes::isProjectionOfPointOnPlane(Eigen::Vector3f point, int plane_nr) {
  pcl::PointNormal plane = plane_centroid_with_normals_.points[plane_nr];
  float tol = 0.5;
  cgal::Plane intersection_plane =
      cgal::Plane(cgal::Point(plane.x, plane.y, plane.z),
                  cgal::Vector(plane.normal_x, plane.normal_y, plane.normal_z));
  cgal::Line normal_through_point =
      cgal::Line(cgal::Point(point[0], point[1], point[2]),
                 cgal::Vector(plane.normal_x, plane.normal_y, plane.normal_z));
  cgal::Point intersection_point;
  CGAL::Object result_intersection_point;

  result_intersection_point = CGAL::intersection(intersection_plane, normal_through_point);
  if (!CGAL::assign(intersection_point, result_intersection_point)) {
    std::cout << "MapPlanes: Error, there should be an intersection points" << std::endl;
    return false;
  };

  if (plane_boundaries_[0][plane_nr] - tol < intersection_point.x() &&
      intersection_point.x() < plane_boundaries_[1][plane_nr] + tol &&
      plane_boundaries_[2][plane_nr] - tol < intersection_point.y() &&
      intersection_point.y() < plane_boundaries_[3][plane_nr] + tol &&
      plane_boundaries_[4][plane_nr] - tol < intersection_point.z() &&
      intersection_point.z() < plane_boundaries_[5][plane_nr] + tol) {
    return true;
  }
  return false;
};

void BoundedPlanes::getPlaneInformations(pcl::PointCloud<pcl::PointNormal> &planes_out,
                                         std::vector<std::vector<float>> &boundaries_out) {
  planes_out.clear();
  for (auto plane : plane_centroid_with_normals_) planes_out.push_back(plane);
  boundaries_out = plane_boundaries_;
};

pcl::PointCloud<pcl::PointNormal> BoundedPlanes::getPlaneCentroidsAndNormals() {
  return plane_centroid_with_normals_;
}

int BoundedPlanes::getPlaneNumber() { return plane_centroid_with_normals_.size(); };

void BoundedPlanes::dispAllPlanes() {
  int plane_nr = 0;
  std::cout << "Print all information about planes ..." << std::endl;
  for (auto plane : plane_centroid_with_normals_) {
    std::cout << "plane nr. " << plane_nr << " x: " << plane.x << " y: " << plane.y
              << "z : " << plane.z << std::endl;
    std::cout << "normal x: " << plane.normal_x << " normal y: " << plane.normal_y
              << " normal z: " << plane.normal_z << std::endl;
    std::cout << "boundaries: xmin: " << plane_boundaries_[0][plane_nr]
              << " boundaries: xmax: " << plane_boundaries_[1][plane_nr] << std::endl;
    std::cout << "boundaries: ymin: " << plane_boundaries_[2][plane_nr]
              << " boundaries: ymax: " << plane_boundaries_[3][plane_nr] << std::endl;
    std::cout << "boundaries: zmin: " << plane_boundaries_[4][plane_nr]
              << " boundaries: zmax: " << plane_boundaries_[5][plane_nr] << std::endl;
    plane_nr++;
  }
};

void BoundedPlanes::saveToYamlFile(std::string file_name) {
  // Save data in yaml file
  std::string plane_name = "map_plane_nr_";
  std::string actual_plane_name;
  YAML::Node node;
  node["size"] = plane_centroid_with_normals_.size();
  for (int plane_nr = 0; plane_nr < plane_centroid_with_normals_.size(); ++plane_nr) {
    actual_plane_name = plane_name + std::to_string(plane_nr);
    node[actual_plane_name].push_back(plane_centroid_with_normals_.points[plane_nr].x);
    node[actual_plane_name].push_back(plane_centroid_with_normals_.points[plane_nr].y);
    node[actual_plane_name].push_back(plane_centroid_with_normals_.points[plane_nr].z);
    node[actual_plane_name].push_back(plane_centroid_with_normals_.points[plane_nr].normal_x);
    node[actual_plane_name].push_back(plane_centroid_with_normals_.points[plane_nr].normal_y);
    node[actual_plane_name].push_back(plane_centroid_with_normals_.points[plane_nr].normal_z);
    node[actual_plane_name].push_back(plane_boundaries_[0][plane_nr]);
    node[actual_plane_name].push_back(plane_boundaries_[1][plane_nr]);
    node[actual_plane_name].push_back(plane_boundaries_[2][plane_nr]);
    node[actual_plane_name].push_back(plane_boundaries_[3][plane_nr]);
    node[actual_plane_name].push_back(plane_boundaries_[4][plane_nr]);
    node[actual_plane_name].push_back(plane_boundaries_[5][plane_nr]);
  }
  std::ofstream fout(file_name);
  fout << node;
};

void BoundedPlanes::loadFromYamlFile(std::string file_name) {
  YAML::Node node = YAML::LoadFile(file_name);
  int map_size = node["size"].as<int>();
  std::string actual_plane_name;
  std::string plane_name = "map_plane_nr_";
  pcl::PointNormal map_plane;
  for (int plane_nr = 0; plane_nr < map_size; ++plane_nr) {
    actual_plane_name = plane_name + std::to_string(plane_nr);
    map_plane.x = node[actual_plane_name][0].as<float>();
    map_plane.y = node[actual_plane_name][1].as<float>();
    map_plane.z = node[actual_plane_name][2].as<float>();
    map_plane.normal_x = node[actual_plane_name][3].as<float>();
    map_plane.normal_y = node[actual_plane_name][4].as<float>();
    map_plane.normal_z = node[actual_plane_name][5].as<float>();
    plane_centroid_with_normals_.push_back(map_plane);
    plane_boundaries_[0].push_back(node[actual_plane_name][6].as<float>());
    plane_boundaries_[1].push_back(node[actual_plane_name][7].as<float>());
    plane_boundaries_[2].push_back(node[actual_plane_name][8].as<float>());
    plane_boundaries_[3].push_back(node[actual_plane_name][9].as<float>());
    plane_boundaries_[4].push_back(node[actual_plane_name][10].as<float>());
    plane_boundaries_[5].push_back(node[actual_plane_name][11].as<float>());
  }
};
}  // namespace matching_algorithms
}  // namespace cad_percept