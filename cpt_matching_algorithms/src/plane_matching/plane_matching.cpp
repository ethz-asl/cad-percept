#include "plane_matching/plane_matching.h"

using namespace cad_percept::cgal;

namespace cad_percept {
namespace matching_algorithms {

class PlaneMatch::SortRelativeTriangle {
 public:
  SortRelativeTriangle(){};

  void insert(float input_edges[3], bool input_isline) {
    isline = input_isline;
    if (input_edges[0] < input_edges[1]) {
      if (input_edges[0] < input_edges[2]) {
        if (input_edges[1] < input_edges[2]) {
          edges_[0] = input_edges[2];
          edges_[1] = input_edges[1];
          edges_[2] = input_edges[0];
        } else {
          edges_[0] = input_edges[1];
          edges_[1] = input_edges[2];
          edges_[2] = input_edges[0];
        }
      } else {
        edges_[0] = input_edges[1];
        edges_[1] = input_edges[0];
        edges_[2] = input_edges[2];
      }
    } else {
      if (input_edges[0] > input_edges[2]) {
        if (input_edges[1] < input_edges[2]) {
          edges_[0] = input_edges[0];
          edges_[1] = input_edges[2];
          edges_[2] = input_edges[1];
        } else {
          edges_[0] = input_edges[0];
          edges_[1] = input_edges[1];
          edges_[2] = input_edges[2];
        }
      } else {
        edges_[0] = input_edges[2];
        edges_[1] = input_edges[0];
        edges_[2] = input_edges[1];
      }
    }
  }

  bool getEdges(Eigen::Vector3f &output_edges) {
    output_edges[0] = edges_[0];
    output_edges[1] = edges_[1];
    output_edges[2] = edges_[2];
    return isline;
  };

 private:
  float edges_[3];
  bool isline = false;
};

float PlaneMatch::PRRUS(float (&transformTR)[7],
                        const pcl::PointCloud<pcl::PointNormal> scan_planes, MapPlanes map_planes,
                        std::vector<std::vector<float>> &results) {
  // std::cout << "////////////////////////////////////////////////" << std::endl;
  // std::cout << "////              PRRUS Started             ////" << std::endl;
  // std::cout << "////////////////////////////////////////////////" << std::endl;

  ros::NodeHandle nh_private("~");
  float translation_penalty = nh_private.param<float>("LineSegmentRansacTranErrorPenalty", 20);

  pcl::PointCloud<pcl::PointNormal> map_planes_pc = map_planes.getPlaneCentroidsAndNormals();

  // Seperate planes into groups containing planes with normals mainly in x,y or z directions
  std::vector<int> map_planes_x;
  std::vector<int> map_planes_y;
  std::vector<int> map_planes_z;

  int map_plane_nr = 0;
  for (auto map_plane : map_planes_pc) {
    if (std::abs(map_plane.normal_x) > 0.6) {
      map_planes_x.push_back(map_plane_nr);
    } else if (std::abs(map_plane.normal_y) > 0.6) {
      map_planes_y.push_back(map_plane_nr);
    } else {
      map_planes_z.push_back(map_plane_nr);
    }
    ++map_plane_nr;
  }

  // Create orthogonal groups of planes in map
  std::vector<Eigen::Vector3i> orthogonal_groups_in_map;
  for (auto map_plane_x : map_planes_x) {
    for (auto map_plane_y : map_planes_y) {
      for (auto map_plane_z : map_planes_z) {
        orthogonal_groups_in_map.push_back(Eigen::Vector3i(map_plane_x, map_plane_y, map_plane_z));
      }
    }
  }
  // if (orthogonal_groups_in_map.size() == 0) {
  //   std::cout
  //       << "Can not find transformation, as no group of orthogonal planes in map could be found"
  //       << std::endl;
  // } else {
  //   std::cout << "Found " << orthogonal_groups_in_map.size()
  //             << " groups of orthogonal planes in map" << std::endl;
  // }

  // Find groups of orthogonal planes in scan
  std::vector<Eigen::Vector3i> orthogonal_groups_in_scan;
  Eigen::Vector3f scan_plane_normal[3];
  Eigen::Vector3f cross_product;
  for (int i = 0; i < ((int)scan_planes.size() - 2); ++i) {
    // std::cout << i << std::endl;
    scan_plane_normal[0] =
        Eigen::Vector3f(scan_planes.points[i].normal_x, scan_planes.points[i].normal_y,
                        scan_planes.points[i].normal_z);
    for (int j = i + 1; j < ((int)scan_planes.size() - 1); ++j) {
      scan_plane_normal[1] =
          Eigen::Vector3f(scan_planes.points[j].normal_x, scan_planes.points[j].normal_y,
                          scan_planes.points[j].normal_z);

      // Continue if normals are collinear
      cross_product = scan_plane_normal[0].cross(scan_plane_normal[1]);
      if (cross_product.norm() < 0.6) continue;
      for (int k = j + 1; k < scan_planes.size(); ++k) {
        scan_plane_normal[2] =
            Eigen::Vector3f(scan_planes.points[k].normal_x, scan_planes.points[k].normal_y,
                            scan_planes.points[k].normal_z);
        if (std::abs(cross_product.dot(scan_plane_normal[2])) > 0.6) {
          // If orthogonal set, save combination (all permutatios)
          orthogonal_groups_in_scan.push_back(Eigen::Vector3i(i, j, k));
          orthogonal_groups_in_scan.push_back(Eigen::Vector3i(i, k, j));
          orthogonal_groups_in_scan.push_back(Eigen::Vector3i(j, i, k));
          orthogonal_groups_in_scan.push_back(Eigen::Vector3i(j, k, i));
          orthogonal_groups_in_scan.push_back(Eigen::Vector3i(k, i, j));
          orthogonal_groups_in_scan.push_back(Eigen::Vector3i(k, j, i));
        }
      }
    }
  }
  // if (orthogonal_groups_in_scan.size() == 0) {
  //   std::cout
  //       << "Can not find transformation, as no group of orthogonal planes in scan could be found"
  //       << std::endl;
  // } else {
  //   std::cout << "Found " << orthogonal_groups_in_scan.size()
  //             << " groups of orthogonal planes in scan" << std::endl;
  // }

  // Create assignments which are consistent considering rotation
  std::vector<Eigen::Matrix<int, 2, 3>> assignments;
  std::vector<Eigen::Quaternionf> rotations_of_assignments;
  Eigen::Quaternionf current_quat;
  Eigen::Hyperplane<float, 3> second_rotation_plane;
  Eigen::Vector3f scan_normal[3];
  Eigen::Vector3f map_normal[3];
  Eigen::Vector3f reprojected_map_normal;
  bool is_valid_assignment;
  Eigen::Matrix<int, 2, 3> current_assignment;
  for (auto scan_orth_group : orthogonal_groups_in_scan) {
    for (auto map_orth_group : orthogonal_groups_in_map) {
      scan_normal[0] = Eigen::Vector3f(scan_planes.points[scan_orth_group[0]].normal_x,
                                       scan_planes.points[scan_orth_group[0]].normal_y,
                                       scan_planes.points[scan_orth_group[0]].normal_z);
      scan_normal[1] = Eigen::Vector3f(scan_planes.points[scan_orth_group[1]].normal_x,
                                       scan_planes.points[scan_orth_group[1]].normal_y,
                                       scan_planes.points[scan_orth_group[1]].normal_z);
      scan_normal[2] = Eigen::Vector3f(scan_planes.points[scan_orth_group[2]].normal_x,
                                       scan_planes.points[scan_orth_group[2]].normal_y,
                                       scan_planes.points[scan_orth_group[2]].normal_z);
      map_normal[0] = Eigen::Vector3f(map_planes_pc.points[map_orth_group[0]].normal_x,
                                      map_planes_pc.points[map_orth_group[0]].normal_y,
                                      map_planes_pc.points[map_orth_group[0]].normal_z);
      map_normal[1] = Eigen::Vector3f(map_planes_pc.points[map_orth_group[1]].normal_x,
                                      map_planes_pc.points[map_orth_group[1]].normal_y,
                                      map_planes_pc.points[map_orth_group[1]].normal_z);
      map_normal[2] = Eigen::Vector3f(map_planes_pc.points[map_orth_group[2]].normal_x,
                                      map_planes_pc.points[map_orth_group[2]].normal_y,
                                      map_planes_pc.points[map_orth_group[2]].normal_z);

      // Calculate corresponding quaternion (algorithm from
      // https://stackoverflow.com/questions/19445934/quaternion-from-two-vector-pairs)
      current_quat = Eigen::Quaternionf::FromTwoVectors(scan_normal[1], map_normal[1]);
      reprojected_map_normal = current_quat.inverse() * map_normal[0];
      second_rotation_plane = Eigen::Hyperplane<float, 3>(scan_normal[1], Eigen::Vector3f(0, 0, 0));
      current_quat = (current_quat * Eigen::Quaternionf::FromTwoVectors(
                                         second_rotation_plane.projection(scan_normal[0]),
                                         second_rotation_plane.projection(reprojected_map_normal)))
                         .normalized();
      // Rotate scan_normals
      is_valid_assignment = true;
      for (int i = 0; i < 3 && is_valid_assignment; ++i) {
        scan_normal[i] = current_quat * scan_normal[i];
        is_valid_assignment =
            is_valid_assignment && ((scan_normal[i] - map_normal[i]).norm() < 0.1);
      }
      if (is_valid_assignment) {
        // Add valid assignment
        current_assignment.row(0) = scan_orth_group;
        current_assignment.row(1) = map_orth_group;
        assignments.push_back(current_assignment);
        rotations_of_assignments.push_back(current_quat);
      }
    }
  }

  // if (assignments.size() == 0) {
  //   std::cout << "Can not find transformation, as no consistent rotation could be found"
  //             << std::endl;
  // } else {
  //   std::cout << "Found " << assignments.size() << " possible assignments" << std::endl;
  // }

  // Apply cost function on left free assignments
  float min_error = -1;
  float error = 0;
  float actual_transformation[7];
  int assignment_nr = 0;
  std::vector<float> actual_result;
  for (auto candidate_assignment : assignments) {
    getTranslationError(candidate_assignment, actual_transformation, error,
                        rotations_of_assignments[assignment_nr], scan_planes, map_planes,
                        translation_penalty);
    // actual_result.clear();
    // actual_result.push_back(actual_transformation[0]);
    // actual_result.push_back(actual_transformation[1]);
    // actual_result.push_back(actual_transformation[2]);
    // actual_result.push_back(actual_transformation[3]);
    // actual_result.push_back(actual_transformation[4]);
    // actual_result.push_back(actual_transformation[5]);
    // actual_result.push_back(actual_transformation[6]);
    // actual_result.push_back(error);
    // std::cout << "add trans" << std::endl;
    // results.push_back(actual_result);
    if (error < min_error || min_error < 0) {
      for (int i = 0; i < 7; ++i) {
        transformTR[i] = actual_transformation[i];
      }
      min_error = error;
      // std::cout << "error: " << min_error << std::endl;
      // std::cout << candidate_assignment << std::endl;
    }
    ++assignment_nr;
  }
  // std::cout << "final error: " << min_error << std::endl;
  return min_error;
};

void PlaneMatch::getTranslationError(Eigen::Matrix<int, 2, Eigen::Dynamic> assignment,
                                     float (&actual_transformation)[7], float &transl_error,
                                     Eigen::Quaternionf rotation,
                                     const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                     MapPlanes map_planes, float translation_penalty) {
  transl_error = 0;
  // Write rotation to resulting transformation
  actual_transformation[3] = rotation.w();
  actual_transformation[4] = rotation.x();
  actual_transformation[5] = rotation.y();
  actual_transformation[6] = rotation.z();

  pcl::PointCloud<pcl::PointNormal> map_planes_pc = map_planes.getPlaneCentroidsAndNormals();
  pcl::PointCloud<pcl::PointNormal> reduced_map_planes;
  pcl::PointCloud<pcl::PointNormal> copy_scan_planes;

  // Find translation (only with assignment)
  std::vector<std::pair<int, int>> plane_assignment;
  for (int i = 0; i < assignment.cols(); ++i) {
    copy_scan_planes.push_back(scan_planes.points[assignment(0, i)]);
    reduced_map_planes.push_back(map_planes_pc.points[assignment(1, i)]);
    plane_assignment.push_back(std::make_pair(assignment(0, i), assignment(1, i)));
  }
  std::sort(
      plane_assignment.begin(), plane_assignment.end(),
      [](const std::pair<int, int> &l, const std::pair<int, int> &r) { return l.first > r.first; });

  Eigen::Matrix4f rotation_transf = Eigen::Matrix4f::Identity();
  rotation_transf.block(0, 0, 3, 3) = rotation.matrix();
  pcl::transformPointCloud(copy_scan_planes, copy_scan_planes, rotation_transf);

  // Mean filter for translation
  std::vector<double> translation_x;
  std::vector<double> translation_y;
  std::vector<double> translation_z;
  for (int plane_nr = 0; plane_nr < copy_scan_planes.size(); ++plane_nr) {
    if (std::abs(reduced_map_planes.points[plane_nr].normal_x) > 0.6) {
      translation_x.push_back(reduced_map_planes.points[plane_nr].x -
                              copy_scan_planes.points[plane_nr].x);
    }
    if (std::abs(reduced_map_planes.points[plane_nr].normal_y) > 0.6) {
      translation_y.push_back(reduced_map_planes.points[plane_nr].y -
                              copy_scan_planes.points[plane_nr].y);
    }
    if (std::abs(reduced_map_planes.points[plane_nr].normal_z) > 0.6) {
      translation_z.push_back(reduced_map_planes.points[plane_nr].z -
                              copy_scan_planes.points[plane_nr].z);
    }
  }
  if (translation_x.size() != 0) {
    actual_transformation[0] =
        std::accumulate(translation_x.begin(), translation_x.end(), 0.0) / translation_x.size();
  } else {
    actual_transformation[0] = 0;
  }
  if (translation_y.size() != 0) {
    actual_transformation[1] =
        std::accumulate(translation_y.begin(), translation_y.end(), 0.0) / translation_y.size();
  } else {
    actual_transformation[1] = 0;
  }
  if (translation_z.size() != 0) {
    actual_transformation[2] =
        std::accumulate(translation_z.begin(), translation_z.end(), 0.0) / translation_z.size();
  } else {
    actual_transformation[2] = 0;
  }

  // Get transformation matrix
  Eigen::Matrix4f actual_transform = Eigen::Matrix4f::Identity();
  Eigen::Vector3f translation =
      Eigen::Vector3f(actual_transformation[0], actual_transformation[1], actual_transformation[2]);
  actual_transform.block(0, 3, 3, 1) = translation;
  actual_transform.block(0, 0, 3, 3) = rotation.matrix();

  copy_scan_planes.clear();
  pcl::transformPointCloudWithNormals(scan_planes, copy_scan_planes, actual_transform);

  std::vector<float> actual_translation_error;
  int map_plane_nr = 0;
  int scan_plane_nr = 0;
  for (auto scan_plane : copy_scan_planes) {
    map_plane_nr = 0;
    actual_translation_error.clear();
    if (scan_plane_nr == plane_assignment.back().first) {
      // Take reference if plane is in matching
      // Only accept plane if it has the same normal
      if (Eigen::Vector3f(map_planes_pc.points[plane_assignment.back().second].normal_x,
                          map_planes_pc.points[plane_assignment.back().second].normal_y,
                          map_planes_pc.points[plane_assignment.back().second].normal_z)
              .dot(Eigen::Vector3f(scan_plane.normal_x, scan_plane.normal_y, scan_plane.normal_z)) >
          0.8) {
        // Only accept point as possible match if projection lies on this plane
        if (map_planes.isProjectionOfPointOnPlane(
                Eigen::Vector3f(scan_plane.x, scan_plane.y, scan_plane.z),
                plane_assignment.back().second)) {
          // Add plane as (single) candidate to error calculation
          actual_translation_error.push_back(
              std::abs((Eigen::Vector3f(scan_plane.x, scan_plane.y, scan_plane.z) -
                        Eigen::Vector3f(map_planes_pc.points[plane_assignment.back().second].x,
                                        map_planes_pc.points[plane_assignment.back().second].y,
                                        map_planes_pc.points[plane_assignment.back().second].z))
                           .dot(Eigen::Vector3f(
                               map_planes_pc.points[plane_assignment.back().second].normal_x,
                               map_planes_pc.points[plane_assignment.back().second].normal_y,
                               map_planes_pc.points[plane_assignment.back().second].normal_z))));
        }
      }
      plane_assignment.pop_back();
    } else {  // search closest plane
      // Only accept plane if it has the same normal
      for (auto map_plane : map_planes_pc) {
        if (Eigen::Vector3f(map_plane.normal_x, map_plane.normal_y, map_plane.normal_z)
                .dot(Eigen::Vector3f(scan_plane.normal_x, scan_plane.normal_y,
                                     scan_plane.normal_z)) < 0.8) {
          map_plane_nr++;
          continue;
        }
        // Only accept point as possible match if projection lies on this plane
        if (!map_planes.isProjectionOfPointOnPlane(
                Eigen::Vector3f(scan_plane.x, scan_plane.y, scan_plane.z), map_plane_nr)) {
          map_plane_nr++;
          continue;
        }
        // Push back possible smallest distance to one of the map planes
        actual_translation_error.push_back(std::abs(
            (Eigen::Vector3f(scan_plane.x, scan_plane.y, scan_plane.z) -
             Eigen::Vector3f(map_plane.x, map_plane.y, map_plane.z))
                .dot(Eigen::Vector3f(map_plane.normal_x, map_plane.normal_y, map_plane.normal_z))));
        map_plane_nr++;
      }
    }
    if (actual_translation_error.size() != 0) {
      transl_error +=
          *std::min_element(actual_translation_error.begin(), actual_translation_error.end());
    } else {
      transl_error += translation_penalty;
    }
    ++scan_plane_nr;
  }
};

void PlaneMatch::IntersectionPatternMatcher(float (&transformTR)[7],
                                            const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                            MapPlanes map_planes) {
  std::cout << "////////////////////////////////////////////////" << std::endl;
  std::cout << "////  Intersection Pattern Matcher Started  ////" << std::endl;
  std::cout << "////////////////////////////////////////////////" << std::endl;

  ros::NodeHandle nh_private("~");
  float parallel_threshold = nh_private.param<float>("IntersectionPatternParallelThreshold", 0.5);
  float threshold_cornerness = nh_private.param<float>("IntersectionPatternCornerThreshold", 0.5);

  // Find intersection points of intersection line of two planes with one other plane
  std::cout << "Search for intersection points in scan and map" << std::endl;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> scan_intersection_points;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> map_intersection_points;
  getprojPlaneIntersectionPoints(scan_intersection_points, parallel_threshold, scan_planes);
  getprojPlaneIntersectionPoints(map_intersection_points, parallel_threshold,
                                 map_planes.getPlaneCentroidsAndNormals());

  // int consider_map_plane = 6;
  // std::cout << "map" << std::endl;
  // for (auto map_points : map_intersection_points) {
  //   std::cout << map_points.size() << std::endl;
  // }

  // std::cout << "map" << std::endl;
  // for (auto map_point : map_intersection_points[consider_map_plane]) {
  //   std::cout << map_point.x << " " << map_point.y << " " << map_point.z << " " << std::endl;
  // }

  std::cout << "Filtered intersection points" << std::endl;
  filterIntersectionPoints(scan_intersection_points);
  filterIntersectionPoints(map_intersection_points);

  // std::cout << "map" << std::endl;
  // for (auto map_point : map_intersection_points[consider_map_plane]) {
  //   std::cout << map_point.x << " " << map_point.y << " " << map_point.z << " " << std::endl;
  // }

  // Remove points on map planes not corresponding to this plane
  std::vector<int> rm_index;
  pcl::ExtractIndices<pcl::PointXYZ> indices_filter;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  int point_index = 0;
  for (int i = 0; i < map_planes.getMapPlaneNumber(); i++) {
    point_index = 0;
    rm_index.clear();
    filter_cloud->clear();
    for (auto map_point : map_intersection_points[i]) {
      if (!map_planes.isProjectionOfPointOnPlane(
              Eigen::Vector3f(map_point.x, map_point.y, map_point.z), point_index)) {
        rm_index.push_back(point_index);
      }
      point_index++;
    }
    boost::shared_ptr<std::vector<int>> inliers_ptr =
        boost::make_shared<std::vector<int>>(rm_index);
    *filter_cloud = map_intersection_points[i];
    indices_filter.setInputCloud(filter_cloud);
    indices_filter.setIndices(inliers_ptr);
    indices_filter.setNegative(true);
    indices_filter.filter(*filter_cloud);
    map_intersection_points[i].clear();
    map_intersection_points[i] = *filter_cloud;
  }

  // std::cout << "map" << std::endl;
  // for (auto point : map_intersection_points) {
  //   std::cout << point.size() << std::endl;
  // }
  // std::cout << "map" << std::endl;
  // for (auto map_point : map_intersection_points[consider_map_plane]) {
  //   std::cout << map_point.x << " " << map_point.y << " " << map_point.z << " " << std::endl;
  // }
  // std::cout << "scan" << std::endl;
  // for (auto map_point : scan_intersection_points[3]) {
  //   std::cout << map_point.x << " " << map_point.y << " " << map_point.z << " " << std::endl;
  // }

  // get strong triangles of intersection points (trivial for given example)
  std::cout << "Constructed all triangles in scan and map" << std::endl;
  std::vector<std::vector<SortRelativeTriangle>> triangles_in_scan_planes;
  getIntersectionCornerTriangle(triangles_in_scan_planes, threshold_cornerness,
                                scan_intersection_points);
  std::vector<std::vector<SortRelativeTriangle>> triangles_in_map_planes;
  getIntersectionCornerTriangle(triangles_in_map_planes, threshold_cornerness,
                                map_intersection_points);

  // std::cout << "scan" << std::endl;
  // for (auto point : triangles_in_scan_planes) {
  //   std::cout << point.size() << std::endl;
  // }
  // std::cout << "map" << std::endl;
  // for (auto point : triangles_in_map_planes) {
  //   std::cout << point.size() << std::endl;
  // }

  // std::cout << "scan" << std::endl;
  // Eigen::Vector3f scan_triangle_sidest;
  // for (auto point : triangles_in_scan_planes[3]) {
  //   if (!point.getEdges(scan_triangle_sidest)) {
  //     std::cout << scan_triangle_sidest[0] << " " << scan_triangle_sidest[1] << " "
  //               << scan_triangle_sidest[2] << std::endl;
  //   } else
  //     std::cout << "line: " << scan_triangle_sidest[0] << std::endl;
  // }
  // std::cout << "map" << std::endl;
  // Eigen::Vector3f map_triangle_sidest;
  // for (auto point : triangles_in_map_planes[consider_map_plane]) {
  //   if (!point.getEdges(map_triangle_sidest)) {
  //     std::cout << map_triangle_sidest[0] << " " << map_triangle_sidest[1] << " "
  //               << map_triangle_sidest[2] << std::endl;
  //   } else
  //     std::cout << "line: " << map_triangle_sidest[0] << std::endl;
  // }

  filterIntersectionPoints(triangles_in_scan_planes);
  filterIntersectionPoints(triangles_in_map_planes);

  // std::cout << "scan" << std::endl;
  // for (auto point : triangles_in_scan_planes) {
  //   std::cout << point.size() << std::endl;
  // }
  // std::cout << "map" << std::endl;
  // for (auto point : triangles_in_map_planes) {
  //   std::cout << point.size() << std::endl;
  // }

  // std::cout << "scan" << std::endl;
  // Eigen::Vector3f scan_triangle_sides;
  // for (auto point : triangles_in_scan_planes[1]) {
  //   if (!point.getEdges(scan_triangle_sides)) {
  //     std::cout << scan_triangle_sides[0] << " " << scan_triangle_sides[1] << " "
  //               << scan_triangle_sides[2] << std::endl;
  //   } else
  //     std::cout << "line: " << scan_triangle_sides[0] << std::endl;
  // }
  // std::cout << "map" << std::endl;
  // Eigen::Vector3f map_triangle_sides;
  // for (auto point : triangles_in_map_planes[0]) {
  //   if (!point.getEdges(map_triangle_sides)) {
  //     std::cout << map_triangle_sides[0] << " " << map_triangle_sides[1] << " "
  //               << map_triangle_sides[2] << std::endl;
  //   } else
  //     std::cout << "line: " << map_triangle_sides[0] << std::endl;
  // }

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> scan_to_map_score;
  findTriangleCorrespondences(scan_to_map_score, triangles_in_scan_planes, triangles_in_map_planes);
  std::cout << scan_to_map_score << std::endl;
}

void PlaneMatch::getprojPlaneIntersectionPoints(
    std::vector<pcl::PointCloud<pcl::PointXYZ>> &tot_plane_intersections, float parallel_threshold,
    const pcl::PointCloud<pcl::PointNormal> planes) {
  Plane cgal_plane;
  Plane cgal_pair_plane;
  Line intersection_line;

  Plane cgal_candidate_plane;
  Point intersection_point;

  tot_plane_intersections.clear();
  pcl::PointCloud<pcl::PointXYZ> candidate_intersection_cloud;

  // Project centroids on plane
  CGAL::Object result_intersection_line;
  CGAL::Object result_intersection_point;
  int candidate_nr = 0;

  // Get intersections of each pair of planes with the plane
  for (auto candidate_plane : planes) {
    candidate_intersection_cloud.clear();
    cgal_candidate_plane =
        Plane(Point(candidate_plane.x, candidate_plane.y, candidate_plane.z),
              Vector(candidate_plane.normal_x, candidate_plane.normal_y, candidate_plane.normal_z));
    for (int plane_nr = 0; plane_nr < planes.size(); ++plane_nr) {
      if (plane_nr == candidate_nr) continue;
      cgal_plane = Plane(
          Point(planes.points[plane_nr].x, planes.points[plane_nr].y, planes.points[plane_nr].z),
          Vector(planes.points[plane_nr].normal_x, planes.points[plane_nr].normal_y,
                 planes.points[plane_nr].normal_z));
      for (int plane_pair_nr = (plane_nr + 1); plane_pair_nr < planes.size(); ++plane_pair_nr) {
        if (plane_pair_nr == plane_nr || plane_pair_nr == candidate_nr) continue;
        cgal_pair_plane = Plane(
            Point(planes.points[plane_pair_nr].x, planes.points[plane_pair_nr].y,
                  planes.points[plane_pair_nr].z),
            Vector(planes.points[plane_pair_nr].normal_x, planes.points[plane_pair_nr].normal_y,
                   planes.points[plane_pair_nr].normal_z));
        // Make sure planes are not too parallel
        if (std::abs(cgal_plane.orthogonal_vector() * cgal_pair_plane.orthogonal_vector()) >
            parallel_threshold)
          continue;

        result_intersection_line = CGAL::intersection(cgal_plane, cgal_pair_plane);
        // Skip plane pair if there is no intersection_line
        if (!CGAL::assign(intersection_line, result_intersection_line)) continue;

        result_intersection_point = CGAL::intersection(cgal_candidate_plane, intersection_line);
        // Skip intersection plane if there is no intersection with candidate plane
        if (!CGAL::assign(intersection_point, result_intersection_point)) continue;

        candidate_intersection_cloud.push_back(
            pcl::PointXYZ(intersection_point.x(), intersection_point.y(), intersection_point.z()));
      }
    }
    ++candidate_nr;
    tot_plane_intersections.push_back(candidate_intersection_cloud);
  }
}

void PlaneMatch::getIntersectionCornerTriangle(
    std::vector<std::vector<SortRelativeTriangle>> &triangles_in_planes, float threshold_cornerness,
    std::vector<pcl::PointCloud<pcl::PointXYZ>> tot_plane_intersections) {
  Point vertex_one;
  Point vertex_two;
  Point vertex_three;
  float triangle_sides[3] = {0};
  float input_triangle_sides[3] = {0};
  SortRelativeTriangle triangle;

  triangles_in_planes = std::vector<std::vector<SortRelativeTriangle>>(
      tot_plane_intersections.size(), std::vector<SortRelativeTriangle>(0));

  int plane_nr = 0;
  for (auto plane_points : tot_plane_intersections) {
    int point_one_in_plane = 0;
    for (auto point_one : plane_points) {
      vertex_one = Point(point_one.x, point_one.y, point_one.z);
      for (int point_two_in_plane = point_one_in_plane + 1;
           point_two_in_plane < plane_points.size(); ++point_two_in_plane) {
        vertex_two = Point(plane_points.points[point_two_in_plane].x,
                           plane_points.points[point_two_in_plane].y,
                           plane_points.points[point_two_in_plane].z);
        // Add a line if only two intersection points are found
        if (plane_points.size() == 2) {
          input_triangle_sides[0] = sqrt((vertex_one - vertex_two).squared_length());
          input_triangle_sides[1] = 0;
          input_triangle_sides[2] = 0;
          triangle.insert(input_triangle_sides, true);
          triangles_in_planes[plane_nr].push_back(triangle);
          break;
        }
        for (int point_three_in_plane = point_two_in_plane + 1;
             point_three_in_plane < plane_points.size(); ++point_three_in_plane) {
          vertex_three = Point(plane_points.points[point_three_in_plane].x,
                               plane_points.points[point_three_in_plane].y,
                               plane_points.points[point_three_in_plane].z);
          triangle_sides[0] = sqrt((vertex_one - vertex_two).squared_length());
          triangle_sides[1] = sqrt((vertex_two - vertex_three).squared_length());
          triangle_sides[2] = sqrt((vertex_one - vertex_three).squared_length());

          if (std::abs((vertex_one - vertex_two) * (vertex_two - vertex_three) /
                       (triangle_sides[0] * triangle_sides[1])) >
              0.9) {  // if points are collinear, assign a line to each
            // Line
            input_triangle_sides[0] = triangle_sides[0];
            input_triangle_sides[1] = 0;
            input_triangle_sides[2] = 0;
            triangle.insert(input_triangle_sides, true);
            triangles_in_planes[plane_nr].push_back(triangle);
            input_triangle_sides[0] = triangle_sides[1];
            triangle.insert(input_triangle_sides, true);
            triangles_in_planes[plane_nr].push_back(triangle);
            input_triangle_sides[0] = triangle_sides[2];
            triangle.insert(input_triangle_sides, true);
            triangles_in_planes[plane_nr].push_back(triangle);
          } else {
            // Not interested in absolut position of Triangle but relative, therefore save
            // relative distance
            input_triangle_sides[0] = sqrt((vertex_one - vertex_two).squared_length());
            input_triangle_sides[1] = sqrt((vertex_two - vertex_three).squared_length());
            input_triangle_sides[2] = sqrt((vertex_one - vertex_three).squared_length());
            triangle.insert(input_triangle_sides, false);
            triangles_in_planes[plane_nr].push_back(triangle);
          }
        }
      }
      ++point_one_in_plane;
    }
    ++plane_nr;
  }
}

void PlaneMatch::findTriangleCorrespondences(
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &score,
    std::vector<std::vector<SortRelativeTriangle>> triangles_in_scan_planes,
    std::vector<std::vector<SortRelativeTriangle>> triangles_in_map_planes) {
  ros::NodeHandle nh_private("~");
  float deviation_tri_offset = nh_private.param<float>("IntersectionPatternTriangleDevOffset", 1);
  float deviation_tri_grade = nh_private.param<float>("IntersectionPatternTriangleDevGrade", 1);
  float deviation_lin_offset = nh_private.param<float>("IntersectionPatternLineDevOffset", 1);
  float deviation_lin_grade = nh_private.param<float>("IntersectionPatternLineDevGrade", 1);

  score = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(
      triangles_in_scan_planes.size(), triangles_in_map_planes.size());
  std::vector<int> voted_scores(triangles_in_map_planes.size(), 0);
  Eigen::Vector3f scan_triangle_sides;
  Eigen::Vector3f map_triangle_sides;
  std::vector<float> triangle_error;
  float additional_score;

  int map_plane_nr = 0;
  int scan_plane_nr = 0;
  std::vector<bool> line_compare;
  int min_index;
  for (auto map_plane : triangles_in_map_planes) {
    scan_plane_nr = 0;
    for (auto scan_plane : triangles_in_scan_planes) {
      // Search best suited scan triangle of a candidate map to a scan triangle
      for (auto map_plane_triangle : map_plane) {
        triangle_error.clear();
        if (!map_plane_triangle.getEdges(map_triangle_sides)) {  // Triangle in Map
          for (auto scan_plane_triangle : scan_plane) {
            if (!scan_plane_triangle.getEdges(scan_triangle_sides)) {  // Triangle in Map and Scan
              triangle_error.push_back((map_triangle_sides - scan_triangle_sides).norm());
              line_compare.push_back(false);
            } else {  // Line in Scan, Triangle in Map
              triangle_error.push_back(
                  Eigen::Vector3f(scan_triangle_sides[0] - map_triangle_sides[0], 0, 0).norm());
              line_compare.push_back(true);
              triangle_error.push_back(
                  Eigen::Vector3f(0, scan_triangle_sides[0] - map_triangle_sides[1], 0).norm());
              line_compare.push_back(true);
              triangle_error.push_back(
                  Eigen::Vector3f(0, 0, scan_triangle_sides[0] - map_triangle_sides[2]).norm());
              line_compare.push_back(true);
            }
          }
        }  // Line in Map
        else {
          for (auto scan_plane_triangle : scan_plane) {
            if (!scan_plane_triangle.getEdges(
                    scan_triangle_sides)) {  // Triangle in Scan and Line in Map
              triangle_error.push_back(
                  Eigen::Vector3f(map_triangle_sides[0] - scan_triangle_sides[0], 0, 0).norm());
              line_compare.push_back(true);
              triangle_error.push_back(
                  Eigen::Vector3f(0, map_triangle_sides[0] - scan_triangle_sides[1], 0).norm());
              line_compare.push_back(true);
              triangle_error.push_back(
                  Eigen::Vector3f(0, 0, map_triangle_sides[0] - scan_triangle_sides[2]).norm());
              line_compare.push_back(true);
            } else {  // Line in Map, Line in Scan
              triangle_error.push_back(
                  Eigen::Vector3f((map_triangle_sides[0] - scan_triangle_sides[0]), 0, 0).norm());
              line_compare.push_back(true);
            }
          }
        }

        if (triangle_error.size() != 0) {
          min_index = std::min_element(triangle_error.begin(), triangle_error.end()) -
                      triangle_error.begin();
          if (line_compare[min_index]) {
            // Line Evaluation
            additional_score = deviation_lin_offset +
                               deviation_lin_offset * std::max(-1.0f, -triangle_error[min_index] /
                                                                          deviation_lin_grade);
          } else {
            // Triangle Evaluation
            additional_score = deviation_tri_offset +
                               deviation_tri_offset * std::max(-1.0f, -triangle_error[min_index] /
                                                                          deviation_tri_grade);
          }
          voted_scores[map_plane_nr] += 1;
          score(scan_plane_nr, map_plane_nr) += additional_score;
        }
      }
      scan_plane_nr++;
    }
    map_plane_nr++;
  }

  // Scale vote according to map plane feature number
  for (int i = 0; i < triangles_in_scan_planes.size(); i++) {
    for (int j = 0; j < triangles_in_map_planes.size(); j++) {
      if (triangles_in_map_planes[j].size() != 0) {
        score(i, j) = (int)(score(i, j) / (triangles_in_map_planes[j].size()) * 1000);
      }
    }
  }
}

void PlaneMatch::filterIntersectionPoints(
    std::vector<pcl::PointCloud<pcl::PointXYZ>> &intersection_points) {
  ros::NodeHandle nh_private("~");
  float search_radius = nh_private.param<float>("IntersectionPatternCoarseness", 1);
  float environment_max_size = nh_private.param<float>("IntersectionPatternMaxSize", 100);

  pcl::UniformSampling<pcl::PointXYZ> voxel_filter;
  voxel_filter.setRadiusSearch(search_radius);

  pcl::PassThrough<pcl::PointXYZ> passfilter;
  passfilter.setFilterLimits(-environment_max_size, environment_max_size);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  for (int plane_nr = 0; plane_nr < intersection_points.size(); ++plane_nr) {
    filter_cloud->clear();
    *filter_cloud = intersection_points[plane_nr];

    voxel_filter.setInputCloud(filter_cloud);
    voxel_filter.filter(*filter_cloud);
    passfilter.setInputCloud(filter_cloud);
    passfilter.setFilterFieldName("x");
    passfilter.filter(*filter_cloud);
    passfilter.setFilterFieldName("y");
    passfilter.filter(*filter_cloud);
    passfilter.setFilterFieldName("z");
    passfilter.filter(*filter_cloud);

    intersection_points[plane_nr].clear();
    intersection_points[plane_nr] = *filter_cloud;
  }
}

void PlaneMatch::filterIntersectionPoints(
    std::vector<std::vector<SortRelativeTriangle>> &triangles_in_planes) {
  // Remove triangles which can't be seen by the LiDAR due to range
  float triangle_side_max_size = 50;

  pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointXYZI pcl_point;
  SortRelativeTriangle triangle;
  Eigen::Vector3f triangle_sides;
  float edges[3];
  int plane_nr = 0;
  for (auto plane_triangle : triangles_in_planes) {
    filter_cloud->clear();
    for (auto points : plane_triangle) {
      pcl_point.intensity = points.getEdges(triangle_sides);
      pcl_point.x = triangle_sides[0];
      pcl_point.y = triangle_sides[1];
      pcl_point.z = triangle_sides[2];
      filter_cloud->push_back(pcl_point);
    }

    triangles_in_planes[plane_nr].clear();
    for (auto points : *filter_cloud) {
      edges[0] = points.x;
      edges[1] = points.y;
      edges[2] = points.z;
      triangle.insert(edges, (bool)points.intensity);
      triangles_in_planes[plane_nr].push_back(triangle);
    }
    plane_nr++;
  }
}

void PlaneMatch::LineSegmentRansac(float (&transformTR)[7],
                                   const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                   MapPlanes map_planes) {
  std::cout << "////////////////////////////////////////////////" << std::endl;
  std::cout << "////       Line Segment RANSAC Started      ////" << std::endl;
  std::cout << "////////////////////////////////////////////////" << std::endl;

  ros::NodeHandle nh_private("~");
  int max_size_subset = nh_private.param<int>("LineSegmentRansacSubsetSize", 100);
  float min_error_threshold = nh_private.param<float>("LineSegmentRansacErrorThreshold", 0.5);

  std::vector<SegmentedLine> map_lines;
  std::vector<SegmentedLine> scan_lines;

  // for (auto norm_point : scan_planes) {
  //   std::cout << "point on plane " << norm_point.x << " " << norm_point.y << " " <<
  //   norm_point.z
  //             << std::endl;
  //   std::cout << "normal of plane " << norm_point.normal_x << " " << norm_point.normal_y << " "
  //             << norm_point.normal_z << std::endl;
  // }
  pcl::PointCloud<pcl::PointNormal> map_planes_pc = map_planes.getPlaneCentroidsAndNormals();

  getSegmentedLines(map_lines, map_planes_pc, true, map_planes);
  getSegmentedLines(scan_lines, scan_planes, false, map_planes);

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> match_score;
  getMatchProbLines(map_lines, scan_lines, match_score);

  std::vector<SegmentedLineAssignment> line_assignments;
  SegmentedLineAssignment line_assignment;

  for (int scan_line_nr = 0; scan_line_nr < scan_lines.size(); ++scan_line_nr) {
    for (int map_line_nr = 0; map_line_nr < map_lines.size(); ++map_line_nr) {
      line_assignment = {scan_line_nr, map_line_nr, match_score(scan_line_nr, map_line_nr)};
      line_assignments.push_back(line_assignment);
    }
  }
  std::sort(line_assignments.begin(), line_assignments.end(),
            [](const SegmentedLineAssignment &i, const SegmentedLineAssignment &j) {
              return i.match_score > j.match_score;
            });

  // RANSAC
  SegmentedLine candidate_scan_line_one;
  SegmentedLine candidate_scan_line_two;
  SegmentedLine candidate_map_line_one;
  SegmentedLine candidate_map_line_two;
  Eigen::Matrix<int, 2, 4> assignment;
  float map_transformation_score;
  float scan_transformation_score;
  float transform_score_threshold = 0.4;
  bool possible_comb_map = false;
  bool possible_comb_scan = false;
  bool duplicated_assignment = false;
  float error = 0;
  Eigen::Matrix<int, 2, 4> max_assignement;
  float min_error = -1;
  float actual_transformation[7];

  for (int i = 0; i < line_assignments.size(); ++i) {
    candidate_scan_line_one = scan_lines[line_assignments[i].scan_line_nr];
    candidate_map_line_one = map_lines[line_assignments[i].map_line_nr];
    for (int j = i + 1; j < i + max_size_subset && j < line_assignments.size(); ++j) {
      candidate_scan_line_two = scan_lines[line_assignments[j].scan_line_nr];
      candidate_map_line_two = map_lines[line_assignments[j].map_line_nr];

      // First assignment, which is used to check for possible transformation (independent on
      // assignment)
      assignment(0, 0) = candidate_scan_line_one.plane_nr;
      assignment(0, 1) = candidate_scan_line_one.pair_plane_nr;
      assignment(0, 2) = candidate_scan_line_two.plane_nr;
      assignment(0, 3) = candidate_scan_line_two.pair_plane_nr;
      assignment(1, 0) = candidate_map_line_one.plane_nr;
      assignment(1, 1) = candidate_map_line_one.pair_plane_nr;
      assignment(1, 2) = candidate_map_line_two.plane_nr;
      assignment(1, 3) = candidate_map_line_two.pair_plane_nr;

      // Make sure all planes are assigned once to another plane
      duplicated_assignment = false;
      for (int candidate_nr = 0; candidate_nr < 4; ++candidate_nr) {
        for (int pair_candidate_nr = candidate_nr + 1; pair_candidate_nr < 4; ++pair_candidate_nr) {
          if (assignment(0, candidate_nr) == assignment(0, pair_candidate_nr))
            duplicated_assignment = true;
          if (assignment(1, candidate_nr) == assignment(1, pair_candidate_nr))
            duplicated_assignment = true;
        }
      }
      if (duplicated_assignment) continue;

      // Make sure its possible to find transformation for at least one assignment and if
      // assignment makes sense, otherwise skip
      possible_comb_map = false;
      possible_comb_scan = false;
      for (int comb_nr = 0; comb_nr < 3; ++comb_nr) {
        map_transformation_score = std::abs(
            Eigen::Vector3f(map_planes_pc.points[assignment(1, (comb_nr + 1) % 4)].normal_x,
                            map_planes_pc.points[assignment(1, (comb_nr + 1) % 4)].normal_y,
                            map_planes_pc.points[assignment(1, (comb_nr + 1) % 4)].normal_z)
                .cross(Eigen::Vector3f(
                    map_planes_pc.points[assignment(1, (comb_nr + 2) % 4)].normal_x,
                    map_planes_pc.points[assignment(1, (comb_nr + 2) % 4)].normal_y,
                    map_planes_pc.points[assignment(1, (comb_nr + 2) % 4)].normal_z))
                .dot(Eigen::Vector3f(
                    map_planes_pc.points[assignment(1, (comb_nr + 3) % 4)].normal_x,
                    map_planes_pc.points[assignment(1, (comb_nr + 3) % 4)].normal_y,
                    map_planes_pc.points[assignment(1, (comb_nr + 3) % 4)].normal_z)));
        if (map_transformation_score > transform_score_threshold) possible_comb_map = true;
        scan_transformation_score =
            std::abs(Eigen::Vector3f(scan_planes.points[assignment(0, (comb_nr + 1) % 4)].normal_x,
                                     scan_planes.points[assignment(0, (comb_nr + 1) % 4)].normal_y,
                                     scan_planes.points[assignment(0, (comb_nr + 1) % 4)].normal_z)
                         .cross(Eigen::Vector3f(
                             scan_planes.points[assignment(0, (comb_nr + 2) % 4)].normal_x,
                             scan_planes.points[assignment(0, (comb_nr + 2) % 4)].normal_y,
                             scan_planes.points[assignment(0, (comb_nr + 2) % 4)].normal_z))
                         .dot(Eigen::Vector3f(
                             scan_planes.points[assignment(0, (comb_nr + 3) % 4)].normal_x,
                             scan_planes.points[assignment(0, (comb_nr + 3) % 4)].normal_y,
                             scan_planes.points[assignment(0, (comb_nr + 3) % 4)].normal_z)));
        if (scan_transformation_score > transform_score_threshold) possible_comb_scan = true;
      }
      if (!(possible_comb_map && possible_comb_scan)) continue;

      // Try out different possible assignments
      getAssignmentError(assignment, error, actual_transformation, scan_planes, map_planes);
      if (0 < error && (error < min_error || min_error < 0)) {
        for (int s = 0; s < 7; ++s) {
          transformTR[s] = actual_transformation[s];
        }
        min_error = error;
        std::cout << "error: " << min_error << std::endl;
        std::cout << "scan/map: ";
        std::cout << assignment(0, 0) << "/" << assignment(1, 0) << " ";
        std::cout << assignment(0, 1) << "/" << assignment(1, 1) << " ";
        std::cout << assignment(0, 2) << "/" << assignment(1, 2) << " ";
        std::cout << assignment(0, 3) << "/" << assignment(1, 3) << " " << std::endl;
      }

      assignment(0, 0) = candidate_scan_line_one.pair_plane_nr;
      assignment(0, 1) = candidate_scan_line_one.plane_nr;
      assignment(0, 2) = candidate_scan_line_two.plane_nr;
      assignment(0, 3) = candidate_scan_line_two.pair_plane_nr;
      assignment(1, 0) = candidate_map_line_one.plane_nr;
      assignment(1, 1) = candidate_map_line_one.pair_plane_nr;
      assignment(1, 2) = candidate_map_line_two.plane_nr;
      assignment(1, 3) = candidate_map_line_two.pair_plane_nr;
      getAssignmentError(assignment, error, actual_transformation, scan_planes, map_planes);
      if (0 < error && (error < min_error || min_error < 0)) {
        for (int s = 0; s < 7; ++s) {
          transformTR[s] = actual_transformation[s];
        }
        min_error = error;
        std::cout << "error: " << min_error << std::endl;
        std::cout << "scan/map: ";
        std::cout << assignment(0, 0) << "/" << assignment(1, 0) << " ";
        std::cout << assignment(0, 1) << "/" << assignment(1, 1) << " ";
        std::cout << assignment(0, 2) << "/" << assignment(1, 2) << " ";
        std::cout << assignment(0, 3) << "/" << assignment(1, 3) << " " << std::endl;
      }
      assignment(0, 0) = candidate_scan_line_one.plane_nr;
      assignment(0, 1) = candidate_scan_line_one.pair_plane_nr;
      assignment(0, 2) = candidate_scan_line_two.pair_plane_nr;
      assignment(0, 3) = candidate_scan_line_two.plane_nr;
      assignment(1, 0) = candidate_map_line_one.plane_nr;
      assignment(1, 1) = candidate_map_line_one.pair_plane_nr;
      assignment(1, 2) = candidate_map_line_two.plane_nr;
      assignment(1, 3) = candidate_map_line_two.pair_plane_nr;
      getAssignmentError(assignment, error, actual_transformation, scan_planes, map_planes);
      if (0 < error && (error < min_error || min_error < 0)) {
        for (int s = 0; s < 7; ++s) {
          transformTR[s] = actual_transformation[s];
        }
        min_error = error;
        std::cout << "error: " << min_error << std::endl;
        std::cout << "scan/map: ";
        std::cout << assignment(0, 0) << "/" << assignment(1, 0) << " ";
        std::cout << assignment(0, 1) << "/" << assignment(1, 1) << " ";
        std::cout << assignment(0, 2) << "/" << assignment(1, 2) << " ";
        std::cout << assignment(0, 3) << "/" << assignment(1, 3) << " " << std::endl;
      }
      assignment(0, 0) = candidate_scan_line_one.pair_plane_nr;
      assignment(0, 1) = candidate_scan_line_one.plane_nr;
      assignment(0, 2) = candidate_scan_line_two.pair_plane_nr;
      assignment(0, 3) = candidate_scan_line_two.plane_nr;
      assignment(1, 0) = candidate_map_line_one.plane_nr;
      assignment(1, 1) = candidate_map_line_one.pair_plane_nr;
      assignment(1, 2) = candidate_map_line_two.plane_nr;
      assignment(1, 3) = candidate_map_line_two.pair_plane_nr;
      getAssignmentError(assignment, error, actual_transformation, scan_planes, map_planes);
      if (0 < error && (error < min_error || min_error < 0)) {
        for (int s = 0; s < 7; ++s) {
          transformTR[s] = actual_transformation[s];
        }
        min_error = error;
        std::cout << "error: " << min_error << std::endl;
        std::cout << "scan/map: ";
        std::cout << assignment(0, 0) << "/" << assignment(1, 0) << " ";
        std::cout << assignment(0, 1) << "/" << assignment(1, 1) << " ";
        std::cout << assignment(0, 2) << "/" << assignment(1, 2) << " ";
        std::cout << assignment(0, 3) << "/" << assignment(1, 3) << " " << std::endl;
      }
    }
    if (min_error < min_error_threshold && min_error > 0) break;
  }
  if (min_error < 0)
    std::cout << "Couldn't estimate transformation as no valid orthogonal combination of planes "
                 "could be found"
              << std::endl;
  std::cout << "final error: " << min_error << std::endl;
};

void PlaneMatch::getAssignmentError(Eigen::Matrix<int, 2, Eigen::Dynamic> assignment,
                                    float &transform_error, float (&transform)[7],
                                    const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                    MapPlanes map_planes) {
  ros::NodeHandle nh_private("~");

  float rotation_consistency_threshold =
      nh_private.param<float>("LineSegmentRansacRotConsistThreshold", 0.4);
  float translation_penalty = nh_private.param<float>("LineSegmentRansacTranErrorPenalty", 20);
  float max_trans_along_plane = nh_private.param<float>("LineSegmentRansacTranAlongPlane", 6);
  float translation_norm_threshold =
      nh_private.param<float>("LineSegmentRansacTranErrorNormThreshold", 0.7);

  std::vector<int> actual_plane_assignement;
  std::vector<int> scan_planes_nr;
  std::vector<int> map_planes_nr;
  for (int assignment_nr = 0; assignment_nr < assignment.cols(); ++assignment_nr) {
    actual_plane_assignement.push_back(assignment_nr);
    scan_planes_nr.push_back(assignment(0, assignment_nr));
    map_planes_nr.push_back(assignment(1, assignment_nr));
  }

  // if (scan_planes_nr[0] == 0 && scan_planes_nr[1] == 1 && scan_planes_nr[2] == 2 &&
  //     scan_planes_nr[3] == 6 && map_planes_nr[0] == 6 && map_planes_nr[1] == 21 &&
  //     map_planes_nr[2] == 9 && map_planes_nr[3] == 17) {
  //   std::cout << "here" << std::endl;
  // }
  // std::cout << "////////////////////////////" << std::endl;
  // scan_planes_nr[0] = 3;
  // scan_planes_nr[1] = 2;
  // scan_planes_nr[2] = 5;
  // scan_planes_nr[3] = 4;
  // map_planes_nr[0] = 0;
  // map_planes_nr[1] = 4;
  // map_planes_nr[2] = 17;
  // map_planes_nr[3] = 18;

  // bool check = (scan_planes_nr[0] == 1 && scan_planes_nr[1] == 7 && scan_planes_nr[2] == 1 &&
  //               scan_planes_nr[3] == 4) &&
  //              (map_planes_nr[0] == 0 && map_planes_nr[1] == 5 && map_planes_nr[2] == 0 &&
  //               map_planes_nr[3] == 1);
  // if (check) std::cout << "Assignment found" << std::endl;

  // std::cout << "scan" << scan_planes_nr[0] << " " << scan_planes_nr[1] << " " <<
  // scan_planes_nr[2]
  //           << " " << scan_planes_nr[3] << std::endl;
  // std::cout << map_planes_nr[0] << " " << map_planes_nr[1] << " " << map_planes_nr[2] << " "
  //           << map_planes_nr[3] << std::endl;

  // Check if calculation of transformation is possible and which planes to use
  float transformation_score;
  float map_transformation_score;
  float scan_transformation_score;
  Eigen::Matrix4f actual_transform = Eigen::Matrix4f::Identity();
  Eigen::Vector3f translation;
  Eigen::Quaternionf q;
  float rotation_error;
  transform_error = 0;

  pcl::PointCloud<pcl::PointNormal> reduced_map_planes, reduced_scan_planes;
  pcl::PointCloud<pcl::PointNormal> map_planes_pc = map_planes.getPlaneCentroidsAndNormals();

  // Calculate transformation using the four candidates
  reduced_scan_planes.clear();
  reduced_map_planes.clear();

  for (int j = 0; j < scan_planes_nr.size(); ++j) {
    reduced_scan_planes.push_back(scan_planes.points[scan_planes_nr[j]]);
    reduced_map_planes.push_back(map_planes_pc.points[map_planes_nr[j]]);
  }

  transformAverage(transform, actual_plane_assignement, reduced_scan_planes, reduced_map_planes);

  // Evaluate transformation
  actual_transform = Eigen::Matrix4f::Identity();
  translation = Eigen::Vector3f(transform[0], transform[1], transform[2]);
  q = Eigen::Quaternionf(transform[3], transform[4], transform[5], transform[6]);
  actual_transform.block(0, 0, 3, 3) = q.matrix();
  actual_transform.block(0, 3, 3, 1) = translation;

  pcl::transformPointCloudWithNormals(reduced_scan_planes, reduced_scan_planes, actual_transform);

  // std::cout << "candidate_scan_transform" << std::endl;
  // for (auto norm_point : reduced_scan_planes) {
  //   std::cout << "point on plane " << norm_point.x << " " << norm_point.y << " " <<
  //   norm_point.z
  //             << std::endl;
  //   std::cout << "normal of plane " << norm_point.normal_x << " " << norm_point.normal_y << " "
  //             << norm_point.normal_z << std::endl;
  // }

  rotation_error = 0;
  transform_error = 0;
  int plane_nr = 0;
  for (auto scan_plane : reduced_scan_planes) {
    // Check if plane centroid lies on corresponding plane (should be replaced by projection on
    // plane)
    // std::cout << "here" << std::endl;
    // std::cout << map_planes_nr[plane_nr] << std::endl;
    // std::cout << Eigen::Vector3f(scan_plane.x, scan_plane.y, scan_plane.z) << std::endl;
    // std::cout << map_planes.isProjectionOfPointOnPlane(
    //                  Eigen::Vector3f(scan_plane.x, scan_plane.y, scan_plane.z),
    //                  map_planes_nr[plane_nr])
    //           << std::endl;
    if (!map_planes.isProjectionOfPointOnPlane(
            Eigen::Vector3f(scan_plane.x, scan_plane.y, scan_plane.z), map_planes_nr[plane_nr])) {
      transform_error += translation_penalty;
    } else {
      // Point on plane
      transform_error +=
          std::abs((Eigen::Vector3f(reduced_scan_planes.points[plane_nr].x,
                                    reduced_scan_planes.points[plane_nr].y,
                                    reduced_scan_planes.points[plane_nr].z) -
                    Eigen::Vector3f(reduced_map_planes.points[plane_nr].x,
                                    reduced_map_planes.points[plane_nr].y,
                                    reduced_map_planes.points[plane_nr].z))
                       .dot(Eigen::Vector3f(reduced_map_planes.points[plane_nr].normal_x,
                                            reduced_map_planes.points[plane_nr].normal_y,
                                            reduced_map_planes.points[plane_nr].normal_z)));
    }
    rotation_error += 1 - (Eigen::Vector3f(reduced_scan_planes.points[plane_nr].normal_x,
                                           reduced_scan_planes.points[plane_nr].normal_y,
                                           reduced_scan_planes.points[plane_nr].normal_z)
                               .dot(Eigen::Vector3f(reduced_map_planes.points[plane_nr].normal_x,
                                                    reduced_map_planes.points[plane_nr].normal_y,
                                                    reduced_map_planes.points[plane_nr].normal_z)));
    plane_nr++;
  }
  // Reject assignment if it does not result in a consistent rotation
  if (rotation_error > rotation_consistency_threshold) {
    // std::cout << rotation_error << std::endl;
    transform_error = -1;
    return;
  }
  // std::cout << rotation_error << std::endl;
  // std::cout << transform_error << std::endl;

  // Get additional information from other scan planes (without assignment)
  reduced_scan_planes.clear();
  reduced_map_planes.clear();
  std::vector<int> map_planes_index;
  for (int i = 0; i < map_planes.getMapPlaneNumber(); ++i) {
    if (i == map_planes_nr[0] || i == map_planes_nr[1] || i == map_planes_nr[2] ||
        i == map_planes_nr[3])
      continue;
    reduced_map_planes.push_back(map_planes_pc.points[i]);
    map_planes_index.push_back(i);
  }
  for (int i = 0; i < scan_planes.size(); ++i) {
    if (i == scan_planes_nr[0] || i == scan_planes_nr[1] || i == scan_planes_nr[2] ||
        i == scan_planes_nr[3])
      continue;
    reduced_scan_planes.push_back(scan_planes.points[i]);
  }

  pcl::transformPointCloudWithNormals(reduced_scan_planes, reduced_scan_planes, actual_transform);

  std::vector<float> actual_translation_error;
  int map_plane_nr = 0;
  for (auto scan_plane : reduced_scan_planes) {
    actual_translation_error.clear();
    map_plane_nr = 0;
    for (auto map_plane : reduced_map_planes) {
      if (Eigen::Vector3f(map_plane.normal_x, map_plane.normal_y, map_plane.normal_z)
              .dot(Eigen::Vector3f(scan_plane.normal_x, scan_plane.normal_y, scan_plane.normal_z)) <
          0.8) {
        map_plane_nr++;
        continue;
      }
      if (!map_planes.isProjectionOfPointOnPlane(
              Eigen::Vector3f(scan_plane.x, scan_plane.y, scan_plane.z),
              map_planes_index[map_plane_nr])) {
        map_plane_nr++;
        continue;
      }
      actual_translation_error.push_back(std::abs(
          (Eigen::Vector3f(scan_plane.x, scan_plane.y, scan_plane.z) -
           Eigen::Vector3f(map_plane.x, map_plane.y, map_plane.z))
              .dot(Eigen::Vector3f(map_plane.normal_x, map_plane.normal_y, map_plane.normal_z))));
      // std::cout << "new one" << std::endl;
      // std::cout << scan_plane.x << " " << scan_plane.y << " " << scan_plane.z << std::endl;
      // std::cout << map_plane.x << " " << map_plane.y << " " << map_plane.z << std::endl;
      // std::cout << map_plane.normal_x << " " << map_plane.normal_y << " " << map_plane.normal_z
      //           << std::endl;
      map_plane_nr++;
    }
    if (actual_translation_error.size() != 0) {
      transform_error +=
          *std::min_element(actual_translation_error.begin(), actual_translation_error.end());
    } else {
      transform_error += translation_penalty;
    }
  }
}

void PlaneMatch::getSegmentedLines(std::vector<SegmentedLine> &segmented_lines,
                                   const pcl::PointCloud<pcl::PointNormal> planes, bool ismap,
                                   MapPlanes map_planes) {
  ros::NodeHandle nh_private("~");
  float parallel_threshold = nh_private.param<float>("SegmentedLineParallelThreshold", 0.9);
  float max_distance = nh_private.param<float>("SegmentedLineDistanceThreshold", 100);

  Plane cgal_plane;
  Plane cgal_pair_plane;
  Line candidate_intersection_line;

  Plane cgal_candidate_plane;
  std::vector<Point> intersection_points;
  Point intersection_point;

  segmented_lines.clear();
  SegmentedLine segmentedline;
  std::vector<float> candidate_dist;

  CGAL::Object result_intersection_line;
  CGAL::Object result_intersection_point;

  int plane_nr = 0;
  int intersection_point_nr = 0;
  for (auto plane : planes) {
    cgal_plane = Plane(Point(plane.x, plane.y, plane.z),
                       Vector(plane.normal_x, plane.normal_y, plane.normal_z));
    for (int pair_plane_nr = (plane_nr + 1); pair_plane_nr < planes.size(); ++pair_plane_nr) {
      cgal_pair_plane =
          Plane(Point(planes.points[pair_plane_nr].x, planes.points[pair_plane_nr].y,
                      planes.points[pair_plane_nr].z),
                Vector(planes.points[pair_plane_nr].normal_x, planes.points[pair_plane_nr].normal_y,
                       planes.points[pair_plane_nr].normal_z));

      if (std::abs(cgal_plane.orthogonal_vector() * cgal_pair_plane.orthogonal_vector()) >
          parallel_threshold)
        continue;

      result_intersection_line = CGAL::intersection(cgal_plane, cgal_pair_plane);
      // Skip plane pair if there is no intersection_line
      if (!CGAL::assign(candidate_intersection_line, result_intersection_line)) continue;

      // std::cout << "line: " << plane_nr << " " << pair_plane_nr << " "
      //           << candidate_intersection_line.to_vector()[0] << " "
      //           << candidate_intersection_line.to_vector()[1] << " "
      //           << candidate_intersection_line.to_vector()[2] << std::endl;

      intersection_points.clear();
      for (auto candidate_plane : planes) {
        cgal_candidate_plane = Plane(
            Point(candidate_plane.x, candidate_plane.y, candidate_plane.z),
            Vector(candidate_plane.normal_x, candidate_plane.normal_y, candidate_plane.normal_z));

        result_intersection_point =
            CGAL::intersection(cgal_candidate_plane, candidate_intersection_line);
        // Skip intersection plane if there is no intersection with candidate plane
        if (!CGAL::assign(intersection_point, result_intersection_point)) continue;

        if (ismap && !map_planes.isProjectionOfPointOnPlane(
                         Eigen::Vector3f(intersection_point.x(), intersection_point.y(),
                                         intersection_point.z()),
                         plane_nr))
          continue;
        if (ismap && !map_planes.isProjectionOfPointOnPlane(
                         Eigen::Vector3f(intersection_point.x(), intersection_point.y(),
                                         intersection_point.z()),
                         pair_plane_nr))
          continue;
        intersection_points.push_back(intersection_point);
      }

      // Get distance between intersection points (if they are not too large) as additional
      // description
      candidate_dist.clear();
      for (auto intersection_point : intersection_points) {
        for (int intersection_pair_point = 0; intersection_pair_point < intersection_points.size();
             ++intersection_pair_point) {
          candidate_dist.push_back(
              sqrt((intersection_point - intersection_points[intersection_pair_point])
                       .squared_length()));
          if (candidate_dist.back() > max_distance) {
            candidate_dist.pop_back();
          }
        }
        ++intersection_point_nr;
      }

      // if (ismap) {
      //   std::cout << "lines of " << plane_nr << " " << pair_plane_nr << "includes" <<
      //   std::endl; for (auto intersection_point : intersection_points) {
      //     std::cout << intersection_point.x() << " " << intersection_point.y() << " "
      //               << intersection_point.z() << std::endl;
      //   }
      // }
      if (candidate_dist.size() != 0) {
        std::sort(candidate_dist.begin(), candidate_dist.end());
        segmentedline = {candidate_intersection_line, candidate_dist, plane_nr, pair_plane_nr};
        segmented_lines.push_back(segmentedline);
      }
    }
    ++plane_nr;
  }
}

void PlaneMatch::getMatchProbLines(
    std::vector<SegmentedLine> map_lines, std::vector<SegmentedLine> scan_lines,
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &match_score) {
  ros::NodeHandle nh_private("~");
  float deviation_grade = nh_private.param<float>("SegmentedLineDeviationGrade", 0.5);

  std::vector<float> description_error;
  float min_deviation;

  match_score = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(scan_lines.size(),
                                                                           map_lines.size());
  int map_line_nr = 0;
  int scan_line_nr = 0;
  for (auto map_line : map_lines) {
    scan_line_nr = 0;
    for (auto scan_line : scan_lines) {
      for (auto map_line_description : map_line.line_segments) {
        description_error.clear();
        for (auto scan_line_description : scan_line.line_segments) {
          description_error.push_back(std::abs(map_line_description - scan_line_description));
        }
        min_deviation = *std::min_element(description_error.begin(), description_error.end());
        match_score(scan_line_nr, map_line_nr) +=
            1.0f + std::max(-1.0f, -(powf(min_deviation, 2) / deviation_grade));
      }
      ++scan_line_nr;
    }
    ++map_line_nr;
  }

  for (int i = 0; i < scan_lines.size(); ++i) {
    for (int j = 0; j < map_lines.size(); ++j) {
      if (map_lines[j].line_segments.size() != 0)
        match_score(i, j) = match_score(i, j) / (map_lines[j].line_segments.size());
    }
  }
};

void PlaneMatch::loadExampleSol(float (&transformTR)[7],
                                const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                MapPlanes map_planes) {
  std::vector<int> plane_assignment(scan_planes.size(), 0);
  // // Real data
  // // Solution to example
  // plane_assignment[0] = 14;
  // plane_assignment[1] = 21;
  // plane_assignment[2] = 1;
  // plane_assignment[3] = 0;
  // plane_assignment[4] = 2;
  // plane_assignment[5] = 0;
  // plane_assignment[6] = 14;
  // plane_assignment[7] = 4;

  // Simulated data
  // Simulated data
  // Solution to example with rot 0,0,0,1
  // plane_assignment[0] = 14;
  // plane_assignment[1] = 2;
  // plane_assignment[2] = 0;
  // plane_assignment[3] = 2;
  // plane_assignment[4] = 1;
  // plane_assignment[5] = 0;
  // plane_assignment[6] = 2;
  // plane_assignment[7] = 0;

  // Solution to example with rot 13 -4.5 1 0 0 0.3428978 0.9393727
  plane_assignment[0] = 6;
  plane_assignment[1] = 8;
  plane_assignment[2] = 9;
  plane_assignment[3] = 15;
  plane_assignment[4] = 0;
  plane_assignment[5] = 2;

  transformAverage(transformTR, plane_assignment, scan_planes,
                   map_planes.getPlaneCentroidsAndNormals());
}

void PlaneMatch::transformAverage(float (&transformTR)[7], std::vector<int> plane_assignment,
                                  const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                  const pcl::PointCloud<pcl::PointNormal> map_planes) {
  // std::cout << "Calculate Transformation via plane correspondences" << std::endl;
  // Calculate average quaternion (Map to Lidar), R_Map,Lidar
  Eigen::Quaterniond current_quat;
  Eigen::Vector3d map_normal;
  Eigen::Vector3d map_pair_normal;
  Eigen::Vector3d scan_normal;
  Eigen::Vector3d scan_pair_normal;
  Eigen::Vector3d reprojected_map_normal;
  Eigen::Hyperplane<double, 3> second_rotation_plane;
  std::vector<Eigen::Quaterniond> all_quaternions;

  for (int plane_nr = 0; plane_nr < scan_planes.size(); ++plane_nr) {
    for (int plane_pair_nr = plane_nr + 1; plane_pair_nr < scan_planes.size(); ++plane_pair_nr) {
      scan_normal(0) = scan_planes.points[plane_nr].normal_x;
      scan_normal(1) = scan_planes.points[plane_nr].normal_y;
      scan_normal(2) = scan_planes.points[plane_nr].normal_z;
      scan_pair_normal(0) = scan_planes.points[plane_pair_nr].normal_x;
      scan_pair_normal(1) = scan_planes.points[plane_pair_nr].normal_y;
      scan_pair_normal(2) = scan_planes.points[plane_pair_nr].normal_z;
      map_normal(0) = map_planes.points[plane_assignment[plane_nr]].normal_x;
      map_normal(1) = map_planes.points[plane_assignment[plane_nr]].normal_y;
      map_normal(2) = map_planes.points[plane_assignment[plane_nr]].normal_z;
      map_pair_normal(0) = map_planes.points[plane_assignment[plane_pair_nr]].normal_x;
      map_pair_normal(1) = map_planes.points[plane_assignment[plane_pair_nr]].normal_y;
      map_pair_normal(2) = map_planes.points[plane_assignment[plane_pair_nr]].normal_z;

      // Skip if normals are colinear
      if (std::abs(scan_normal.dot(scan_pair_normal)) > 0.8 ||
          std::abs(map_normal.dot(map_pair_normal)) > 0.8)
        continue;

      // Calculate corresponding quaternion (algorithm from
      // https://stackoverflow.com/questions/19445934/quaternion-from-two-vector-pairs)
      current_quat = Eigen::Quaterniond::FromTwoVectors(scan_pair_normal, map_pair_normal);
      reprojected_map_normal = current_quat.inverse() * map_normal;
      second_rotation_plane =
          Eigen::Hyperplane<double, 3>(scan_pair_normal, Eigen::Vector3d(0, 0, 0));
      current_quat = (current_quat * Eigen::Quaterniond::FromTwoVectors(
                                         second_rotation_plane.projection(scan_normal),
                                         second_rotation_plane.projection(reprojected_map_normal)))
                         .normalized();
      all_quaternions.push_back(current_quat);
    }
  }

  Eigen::MatrixXd all_quat(4, all_quaternions.size());

  int quat_nr = 0;
  for (auto quat : all_quaternions) {
    all_quat(0, quat_nr) = quat.w();
    all_quat(1, quat_nr) = quat.x();
    all_quat(2, quat_nr) = quat.y();
    all_quat(3, quat_nr) = quat.z();
    quat_nr++;
  }

  // M with weight 1
  Eigen::Matrix4d quat_quat = all_quat * all_quat.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(quat_quat);
  Eigen::Vector4d eigenvalues = eigensolver.eigenvalues().col(0);

  int max_ev_index = 0;
  for (int i = 0; i < all_quat.rows(); ++i) {
    if (eigenvalues[i] > eigenvalues[max_ev_index]) {
      max_ev_index = i;
    }
  }
  Eigen::Vector4d res_quat = eigensolver.eigenvectors().col(max_ev_index);

  // res_quat[0] = 0.939;
  // res_quat[1] = 0;
  // res_quat[2] = 0;
  // res_quat[3] = 0.34;

  // Assign to solution
  transformTR[3] = res_quat[0];
  transformTR[4] = res_quat[1];
  transformTR[5] = res_quat[2];
  transformTR[6] = res_quat[3];

  // Find translation
  pcl::PointCloud<pcl::PointNormal> copy_scan_planes;
  Eigen::Matrix4f res_rotation = Eigen::Matrix4f::Identity();
  Eigen::Quaternionf q(transformTR[3], transformTR[4], transformTR[5], transformTR[6]);
  res_rotation.block(0, 0, 3, 3) = q.matrix();
  pcl::transformPointCloud(scan_planes, copy_scan_planes, res_rotation);

  // Mean/Median filter for translation
  std::vector<double> translation_x;
  std::vector<double> translation_y;
  std::vector<double> translation_z;
  int map_index;
  for (int plane_nr = 0; plane_nr < scan_planes.size(); ++plane_nr) {
    map_index = plane_assignment[plane_nr];
    if (std::abs(map_planes.points[map_index].normal_x) > 0.7) {
      translation_x.push_back(map_planes.points[map_index].x - copy_scan_planes.points[plane_nr].x);
    }
    if (std::abs(map_planes.points[map_index].normal_y) > 0.7) {
      translation_y.push_back(map_planes.points[map_index].y - copy_scan_planes.points[plane_nr].y);
    }
    if (std::abs(map_planes.points[map_index].normal_z) > 0.7) {
      translation_z.push_back(map_planes.points[map_index].z - copy_scan_planes.points[plane_nr].z);
    }
  }
  if (translation_x.size() != 0) {
    transformTR[0] =
        std::accumulate(translation_x.begin(), translation_x.end(), 0.0) / translation_x.size();
  } else {
    transformTR[0] = 0;
  }
  if (translation_y.size() != 0) {
    transformTR[1] =
        std::accumulate(translation_y.begin(), translation_y.end(), 0.0) / translation_y.size();
  } else {
    transformTR[1] = 0;
  }
  if (translation_z.size() != 0) {
    transformTR[2] =
        std::accumulate(translation_z.begin(), translation_z.end(), 0.0) / translation_z.size();
  } else {
    transformTR[2] = 0;
  }
  // std::sort(translation_x.begin(), translation_x.end());
  // std::sort(translation_y.begin(), translation_y.end());
  // std::sort(translation_z.begin(), translation_z.end());
  // if (translation_x.size() != 0) transformTR[0] = translation_x[translation_x.size() / 2];
  // if (translation_y.size() != 0) transformTR[1] = translation_y[translation_y.size() / 2];
  // if (translation_z.size() != 0) transformTR[2] = translation_z[translation_z.size() / 2];
};

}  // namespace matching_algorithms
}  // namespace cad_percept