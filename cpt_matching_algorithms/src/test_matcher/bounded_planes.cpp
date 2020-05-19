#include "test_matcher/bounded_planes.h"

namespace cad_percept {
namespace matching_algorithms {

BoundedPlanes::BoundedPlanes(cad_percept::cgal::MeshModel &reference_mesh,
                             std::string cache_folder) {
  ros::NodeHandle nh_private("~");
  float diff_rot_between_planes = nh_private.param<float>("diffRotBetweenPlanes", 0.1);
  float diff_transl_between_planes_along_normal =
      nh_private.param<float>("diffTranslBetweenPlanesAlongNormal", 1);
  float diff_transl_between_planes_along_plane =
      nh_private.param<float>("diffTranslBetweenPlanesAlongPlane", 6);
  float min_area = nh_private.param<float>("minArea", 4);
  float findAllCoplanarFacetseps = nh_private.param<float>("eps", 0.5);

  // Check if there exists a yaml-file to the mesh
  std::string cache_file_name =
      cache_folder + "map_" + std::to_string(reference_mesh.size()) + ".yaml";
  std::ifstream cache_file(cache_file_name);
  if (cache_file.fail()) {
    std::cout << "chached failed" << std::endl;
    // If there is no cached data of the mesh, create a new one
    // Extract planes
    std::unordered_map<std::string, std::string> facetToPlane;
    std::unordered_multimap<std::string, std::string> planeToFacets;

    // Use findAllCoplanarFacets for "coarse" plane extraction
    reference_mesh.findAllCoplanarFacets(&facetToPlane, &planeToFacets, findAllCoplanarFacetseps);

    std::multimap<std::string, std::string> grouped_planeToFacets(planeToFacets.begin(),
                                                                  planeToFacets.end());
    std::string actual_plane_id = "-1";
    cgal::Plane actual_plane;
    cgal::Triangle actual_facet;
    pcl::PointNormal plane;
    std::map<char, std::vector<float>> inlier;

    bool is_new_plane = true;
    Eigen::Vector3d actual_plane_normal;
    Eigen::Vector3d actual_plane_centroid;
    Eigen::Vector3d considered_plane_normal;
    Eigen::Vector3d considered_plane_centroid;
    int considered_plane_nr = 0;

    // Combine detected planes if they are nearly the same
    for (auto facet_of_plane : grouped_planeToFacets) {
      actual_facet = reference_mesh.getTriangle(facet_of_plane.second);
      actual_plane = actual_facet.supporting_plane();
      if (facet_of_plane.first.compare(actual_plane_id) != 0) {
        // New plane
        if (actual_plane_id.compare("-1")) {
          // Find centroid and boundaries of previous plane
          actual_plane_centroid(0) =
              std::accumulate(inlier['x'].begin(), inlier['x'].end(), 0.0) / inlier['x'].size();
          actual_plane_centroid(1) =
              std::accumulate(inlier['y'].begin(), inlier['y'].end(), 0.0) / inlier['y'].size();
          actual_plane_centroid(2) =
              std::accumulate(inlier['z'].begin(), inlier['z'].end(), 0.0) / inlier['z'].size();

          // Check if the same plane has already been detected once
          is_new_plane = true;
          considered_plane_nr = 0;
          for (auto considered_plane : plane_centroid_with_normals_) {
            considered_plane_centroid(0) = considered_plane.x;
            considered_plane_centroid(1) = considered_plane.y;
            considered_plane_centroid(2) = considered_plane.z;
            considered_plane_normal(0) = considered_plane.normal_x;
            considered_plane_normal(1) = considered_plane.normal_y;
            considered_plane_normal(2) = considered_plane.normal_z;
            if ((1 - std::abs(actual_plane_normal.dot(considered_plane_normal))) <
                    diff_rot_between_planes &&
                std::abs((actual_plane_centroid - considered_plane_centroid)
                             .dot(considered_plane_normal)) <
                    diff_transl_between_planes_along_normal &&
                ((actual_plane_centroid - considered_plane_centroid) -
                 (actual_plane_centroid - considered_plane_centroid).dot(considered_plane_normal) *
                     considered_plane_normal)
                        .norm() < diff_transl_between_planes_along_plane) {
              is_new_plane = false;
              break;
            }
            considered_plane_nr++;
          }

          if (is_new_plane) {
            // If the plane is to be considered as a new plane, create a new one
            plane.x = actual_plane_centroid(0);
            plane.y = actual_plane_centroid(1);
            plane.z = actual_plane_centroid(2);

            plane.normal_x = actual_plane_normal(0);
            plane.normal_y = actual_plane_normal(1);
            plane.normal_z = actual_plane_normal(2);

            plane_boundaries_["xmin"].push_back(
                *std::min_element(inlier['x'].begin(), inlier['x'].end()));
            plane_boundaries_["xmax"].push_back(
                *std::max_element(inlier['x'].begin(), inlier['x'].end()));
            plane_boundaries_["ymin"].push_back(
                *std::min_element(inlier['y'].begin(), inlier['y'].end()));
            plane_boundaries_["ymax"].push_back(
                *std::max_element(inlier['y'].begin(), inlier['y'].end()));
            plane_boundaries_["zmin"].push_back(
                *std::min_element(inlier['z'].begin(), inlier['z'].end()));
            plane_boundaries_["zmax"].push_back(
                *std::max_element(inlier['z'].begin(), inlier['z'].end()));

            plane_centroid_with_normals_.push_back(plane);
          } else {
            // Add found plane to already existing one
            // Update plane boundaries
            plane_boundaries_["xmin"][considered_plane_nr] =
                std::min(plane_boundaries_["xmin"][considered_plane_nr],
                         *std::min_element(inlier['x'].begin(), inlier['x'].end()));
            plane_boundaries_["xmax"][considered_plane_nr] =
                std::max(plane_boundaries_["xmax"][considered_plane_nr],
                         *std::max_element(inlier['x'].begin(), inlier['x'].end()));
            plane_boundaries_["ymin"][considered_plane_nr] =
                std::min(plane_boundaries_["ymin"][considered_plane_nr],
                         *std::min_element(inlier['y'].begin(), inlier['y'].end()));
            plane_boundaries_["ymax"][considered_plane_nr] =
                std::max(plane_boundaries_["ymax"][considered_plane_nr],
                         *std::max_element(inlier['y'].begin(), inlier['y'].end()));
            plane_boundaries_["zmin"][considered_plane_nr] =
                std::min(plane_boundaries_["zmin"][considered_plane_nr],
                         *std::min_element(inlier['z'].begin(), inlier['z'].end()));
            plane_boundaries_["zmax"][considered_plane_nr] =
                std::max(plane_boundaries_["zmax"][considered_plane_nr],
                         *std::max_element(inlier['z'].begin(), inlier['z'].end()));
          }
        }
        // Assume that normal of first facet is normal of the plane
        actual_plane_normal =
            cgal::cgalVectorToEigenVector(actual_plane.orthogonal_vector()).normalized();

        inlier['x'].clear();
        inlier['y'].clear();
        inlier['z'].clear();

        actual_plane_id = facet_of_plane.first;
      }
      // Add inlier
      for (int i = 0; i < 3; i++) {
        inlier['x'].push_back(actual_facet.vertex(i).x());
        inlier['y'].push_back(actual_facet.vertex(i).y());
        inlier['z'].push_back(actual_facet.vertex(i).z());
      }
    }

    // Remove too small planes and their boundaries
    std::vector<int> remove_list;
    pcl::ExtractIndices<pcl::PointNormal> indices_filter;
    std::vector<float> cuboid_side_lengths;
    for (int plane_nr = 0; plane_nr < plane_centroid_with_normals_.size(); ++plane_nr) {
      cuboid_side_lengths = {
          (plane_boundaries_["xmax"][plane_nr] - plane_boundaries_["xmin"][plane_nr]),
          (plane_boundaries_["ymax"][plane_nr] - plane_boundaries_["ymin"][plane_nr]),
          (plane_boundaries_["zmax"][plane_nr] - plane_boundaries_["zmin"][plane_nr])};
      std::sort(cuboid_side_lengths.begin(), cuboid_side_lengths.end());
      if (std::abs(cuboid_side_lengths[1] * cuboid_side_lengths[2]) < min_area)
        remove_list.push_back(plane_nr);
    }
    std::vector<std::string> dimensions = {"xmin", "xmax", "ymin", "ymax", "zmin", "zmax"};
    for (auto it = remove_list.rbegin(); it != remove_list.rend(); ++it) {
      for (auto dim : dimensions)
        plane_boundaries_[dim].erase(plane_boundaries_[dim].begin() + *it);
    }
    pcl::PointCloud<pcl::PointNormal>::Ptr plane_centroid_with_normals_ptr(
        new pcl::PointCloud<pcl::PointNormal>());
    *plane_centroid_with_normals_ptr = plane_centroid_with_normals_;
    boost::shared_ptr<std::vector<int>> inliers_ptr =
        boost::make_shared<std::vector<int>>(remove_list);
    indices_filter.setInputCloud(plane_centroid_with_normals_ptr);
    indices_filter.setIndices(inliers_ptr);
    indices_filter.setNegative(true);
    indices_filter.filter(*plane_centroid_with_normals_ptr);
    plane_centroid_with_normals_ = *plane_centroid_with_normals_ptr;

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

  if (plane_boundaries_["xmin"][plane_nr] - tol < intersection_point.x() &&
      intersection_point.x() < plane_boundaries_["xmax"][plane_nr] + tol &&
      plane_boundaries_["ymin"][plane_nr] - tol < intersection_point.y() &&
      intersection_point.y() < plane_boundaries_["ymax"][plane_nr] + tol &&
      plane_boundaries_["zmin"][plane_nr] - tol < intersection_point.z() &&
      intersection_point.z() < plane_boundaries_["zmax"][plane_nr] + tol) {
    return true;
  }
  return false;
};

void BoundedPlanes::getPlaneInformations(
    pcl::PointCloud<pcl::PointNormal> &planes_out,
    std::map<std::string, std::vector<float>> &boundaries_out) {
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
    std::cout << "boundaries: xmin: " << plane_boundaries_["xmin"][plane_nr]
              << " boundaries: xmax: " << plane_boundaries_["xmax"][plane_nr] << std::endl;
    std::cout << "boundaries: ymin: " << plane_boundaries_["ymin"][plane_nr]
              << " boundaries: ymax: " << plane_boundaries_["ymax"][plane_nr] << std::endl;
    std::cout << "boundaries: zmin: " << plane_boundaries_["zmin"][plane_nr]
              << " boundaries: zmax: " << plane_boundaries_["zmax"][plane_nr] << std::endl;
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
    node[actual_plane_name].push_back(plane_boundaries_["xmin"][plane_nr]);
    node[actual_plane_name].push_back(plane_boundaries_["xmax"][plane_nr]);
    node[actual_plane_name].push_back(plane_boundaries_["ymin"][plane_nr]);
    node[actual_plane_name].push_back(plane_boundaries_["ymax"][plane_nr]);
    node[actual_plane_name].push_back(plane_boundaries_["zmin"][plane_nr]);
    node[actual_plane_name].push_back(plane_boundaries_["zmax"][plane_nr]);
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
    plane_boundaries_["xmin"].push_back(node[actual_plane_name][6].as<float>());
    plane_boundaries_["xmax"].push_back(node[actual_plane_name][7].as<float>());
    plane_boundaries_["ymin"].push_back(node[actual_plane_name][8].as<float>());
    plane_boundaries_["ymax"].push_back(node[actual_plane_name][9].as<float>());
    plane_boundaries_["zmin"].push_back(node[actual_plane_name][10].as<float>());
    plane_boundaries_["zmax"].push_back(node[actual_plane_name][11].as<float>());
  }
};
}  // namespace matching_algorithms
}  // namespace cad_percept