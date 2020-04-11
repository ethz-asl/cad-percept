#include "plane_matching/plane_matching.h"

using namespace cad_percept::cgal;

namespace cad_percept {
namespace matching_algorithms {

class PlaneMatch::SortRelativeTriangle {
 public:
  SortRelativeTriangle(){};

  void insert(float input_edges[3], bool input_isline) {
    isline = input_isline;
    if (input_edges[0] > input_edges[1]) {
      if (input_edges[0] > input_edges[2]) {
        edges_[0] = input_edges[0];
        if (input_edges[1] > input_edges[2]) {
          edges_[1] = input_edges[1];
          edges_[2] = input_edges[2];
        } else {
          edges_[1] = input_edges[2];
          edges_[2] = input_edges[1];
        }
      } else {
        edges_[0] = input_edges[1];
        edges_[1] = input_edges[0];
        edges_[2] = input_edges[2];
      }
    } else {
      if (input_edges[0] < input_edges[2]) {
        edges_[0] = input_edges[0];
        if (input_edges[1] > input_edges[2]) {
          edges_[1] = input_edges[2];
          edges_[2] = input_edges[1];
        } else {
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

void PlaneMatch::IntersectionPatternMatcher(float (&transformTR)[7],
                                            const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                            const pcl::PointCloud<pcl::PointNormal> map_planes,
                                            Eigen::Matrix<float, 22, 2> room_boundaries) {
  std::cout << "////////////////////////////////////////////////" << std::endl;
  std::cout << "////  Intersection Pattern Matcher Started  ////" << std::endl;
  std::cout << "////////////////////////////////////////////////" << std::endl;

  ros::NodeHandle nh_private("~");
  float parallel_threshold = nh_private.param<float>("IntersectionPatternParallelThreshold", 0.5);
  float threshold_cornerness = nh_private.param<float>("IntersectionPatternCornerThreshold", 0.5);

  // Find intersection points of intersection line of two planes with one other plane
  std::vector<std::vector<IntersectionCornernessPoint>> scan_intersection_points;
  std::vector<std::vector<IntersectionCornernessPoint>> map_intersection_points;
  getprojPlaneIntersectionPoints(scan_intersection_points, parallel_threshold, scan_planes);
  getprojPlaneIntersectionPoints(map_intersection_points, parallel_threshold, map_planes);
  std::cout << "Searched for intersection points in scan and map" << std::endl;

  std::cout << "scan" << std::endl;
  for (auto point : scan_intersection_points) {
    std::cout << point.size() << std::endl;
  }
  std::cout << "map" << std::endl;
  for (auto point : map_intersection_points) {
    std::cout << point.size() << std::endl;
  }

  std::cout << "scan" << std::endl;
  for (auto point : scan_intersection_points[4]) {
    std::cout << point.intersection_point.x() << " " << point.intersection_point.y() << " "
              << point.intersection_point.z() << std::endl;
  }
  std::cout << "map" << std::endl;
  for (auto point : map_intersection_points[1]) {
    std::cout << point.intersection_point.x() << " " << point.intersection_point.y() << " "
              << point.intersection_point.z() << std::endl;
  }

  filterIntersectionPoints(scan_intersection_points);
  filterIntersectionPoints(map_intersection_points);
  std::cout << "Filtered intersection points" << std::endl;

  std::vector<int> rm_index;
  int point_index = 0;
  for (int i = 1; i < 22; i++) {
    point_index = 0;
    rm_index.clear();
    for (auto map_point : map_intersection_points[i]) {
      if (std::abs(map_planes.points[i].normal_x) > 0.8) {
        if (map_point.intersection_point.y() < (room_boundaries(i, 0) - 0.5) ||
            map_point.intersection_point.y() > room_boundaries(i, 1) + 0.5) {
          rm_index.push_back(point_index);
        }
      }
      if (std::abs(map_planes.points[i].normal_y) > 0.8) {
        if (map_point.intersection_point.x() < (room_boundaries(i, 0) - 0.5) ||
            map_point.intersection_point.x() > room_boundaries(i, 1) + 0.5) {
          rm_index.push_back(point_index);
        }
      }
      point_index++;
    }

    for (auto rm_ids = rm_index.rbegin(); rm_ids != rm_index.rend(); ++rm_ids) {
      map_intersection_points[i].erase(map_intersection_points[i].begin() + *rm_ids);
    }
  }

  std::cout << "scan" << std::endl;
  for (auto point : scan_intersection_points) {
    std::cout << point.size() << std::endl;
  }
  std::cout << "map" << std::endl;
  for (auto point : map_intersection_points) {
    std::cout << point.size() << std::endl;
  }

  std::cout << "scan" << std::endl;
  for (auto point : scan_intersection_points[4]) {
    std::cout << point.intersection_point.x() << " " << point.intersection_point.y() << " "
              << point.intersection_point.z() << std::endl;
  }
  std::cout << "map" << std::endl;
  for (auto point : map_intersection_points[1]) {
    std::cout << point.intersection_point.x() << " " << point.intersection_point.y() << " "
              << point.intersection_point.z() << std::endl;
  }

  // get strong triangles of intersection points (trivial for given example)
  std::vector<std::vector<SortRelativeTriangle>> triangles_in_scan_planes;
  getIntersectionCornerTriangle(triangles_in_scan_planes, threshold_cornerness,
                                scan_intersection_points);
  std::vector<std::vector<SortRelativeTriangle>> triangles_in_map_planes;
  getIntersectionCornerTriangle(triangles_in_map_planes, threshold_cornerness,
                                map_intersection_points);
  std::cout << "Constructed all triangles in scan and map" << std::endl;

  // std::cout << "scan" << std::endl;
  // for (auto point : triangles_in_scan_planes) {
  //   std::cout << point.size() << std::endl;
  // }
  // std::cout << "map" << std::endl;
  // for (auto point : triangles_in_map_planes) {
  //   std::cout << point.size() << std::endl;
  // }

  std::cout << "scan" << std::endl;
  Eigen::Vector3f scan_triangle_sides;
  for (auto point : triangles_in_scan_planes[4]) {
    if (!point.getEdges(scan_triangle_sides)) {
      std::cout << scan_triangle_sides[0] << " " << scan_triangle_sides[1] << " "
                << scan_triangle_sides[2] << std::endl;
    } else
      std::cout << "line: " << scan_triangle_sides[0] << std::endl;
  }
  std::cout << "map" << std::endl;
  Eigen::Vector3f map_triangle_sides;
  for (auto point : triangles_in_map_planes[1]) {
    if (!point.getEdges(map_triangle_sides)) {
      std::cout << map_triangle_sides[0] << " " << map_triangle_sides[1] << " "
                << map_triangle_sides[2] << std::endl;
    } else
      std::cout << "line: " << map_triangle_sides[0] << std::endl;
  }

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> scan_to_map_score;
  findTriangleCorrespondences(scan_to_map_score, triangles_in_scan_planes, triangles_in_map_planes);
  std::cout << scan_to_map_score << std::endl;
}

void PlaneMatch::getprojPlaneIntersectionPoints(
    std::vector<std::vector<IntersectionCornernessPoint>> &tot_plane_intersections,
    float parallel_threshold, const pcl::PointCloud<pcl::PointNormal> planes) {
  Plane cgal_plane;
  Plane cgal_pair_plane;
  Line intersection_line;

  Plane cgal_candidate_plane;
  Point intersection_point;
  std::array<int, 3> used_plane;

  tot_plane_intersections.clear();
  tot_plane_intersections = std::vector<std::vector<IntersectionCornernessPoint>>(
      planes.size(), std::vector<IntersectionCornernessPoint>(0));

  // Project centroids on plane
  CGAL::Object result_intersection_line;
  CGAL::Object result_intersection_point;
  int candidate_nr = 0;

  // Get intersections of each pair of planes with the plane
  for (auto candidate_plane : planes) {
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

        IntersectionCornernessPoint cornerness_point = {
            Point(intersection_point.x(), intersection_point.y(), intersection_point.z()),
            1 - (cgal_plane.orthogonal_vector() * cgal_pair_plane.orthogonal_vector())};
        tot_plane_intersections[candidate_nr].push_back(cornerness_point);
      }
    }
    ++candidate_nr;
  }

  // Sort point list accoring to cornerness starting with highest cornerness
  std::vector<IntersectionCornernessPoint> sorted_list;
  for (auto corner_list : tot_plane_intersections) {
    std::sort(corner_list.begin(), corner_list.end(),
              [](const IntersectionCornernessPoint &i, const IntersectionCornernessPoint &j) {
                return i.cornerness > j.cornerness;
              });
  }
}

void PlaneMatch::getIntersectionCornerTriangle(
    std::vector<std::vector<SortRelativeTriangle>> &triangles_in_planes, float threshold_cornerness,
    std::vector<std::vector<IntersectionCornernessPoint>> tot_plane_intersections) {
  triangles_in_planes = std::vector<std::vector<SortRelativeTriangle>>(
      tot_plane_intersections.size(), std::vector<SortRelativeTriangle>(0));

  Point vertex_one;
  Point vertex_two;
  Point vertex_three;
  float triangle_sides[3] = {0};
  SortRelativeTriangle triangle;

  int plane_nr = 0;
  for (auto plane_points : tot_plane_intersections) {
    for (int point_one_in_plane = 0; point_one_in_plane < plane_points.size() - 1;
         ++point_one_in_plane) {
      if (plane_points[point_one_in_plane].cornerness < threshold_cornerness) break;
      vertex_one = plane_points[point_one_in_plane].intersection_point;
      for (int point_two_in_plane = point_one_in_plane + 1;
           point_two_in_plane < plane_points.size(); ++point_two_in_plane) {
        if (plane_points[point_two_in_plane].cornerness < threshold_cornerness) break;
        vertex_two = plane_points[point_two_in_plane].intersection_point;
        // Only add a line if only two intersection points are found
        if (plane_points.size() == 2) {
          triangle_sides[0] = sqrt((vertex_one - vertex_two).squared_length());
          triangle_sides[1] = 0;
          triangle_sides[2] = 0;
          triangle.insert(triangle_sides, true);

          triangles_in_planes[plane_nr].push_back(triangle);
          break;
        }
        for (int point_three_in_plane = point_two_in_plane + 1;
             point_three_in_plane < plane_points.size(); ++point_three_in_plane) {
          if (plane_points[point_three_in_plane].cornerness < threshold_cornerness) break;
          vertex_three = plane_points[point_three_in_plane].intersection_point;
          // Not interested in absolut position of Triangle but relative, therefore save relative
          // distance
          triangle_sides[0] = sqrt((vertex_one - vertex_two).squared_length());
          triangle_sides[1] = sqrt((vertex_two - vertex_three).squared_length());
          triangle_sides[2] = sqrt((vertex_one - vertex_three).squared_length());
          triangle.insert(triangle_sides, false);

          triangles_in_planes[plane_nr].push_back(triangle);
        }
      }
    }
    ++plane_nr;
  }
}

void PlaneMatch::findTriangleCorrespondences(
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &score,
    std::vector<std::vector<SortRelativeTriangle>> triangles_in_scan_planes,
    std::vector<std::vector<SortRelativeTriangle>> triangles_in_map_planes) {
  score = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(
      triangles_in_scan_planes.size(), triangles_in_map_planes.size());

  ros::NodeHandle nh_private("~");
  float deviation_tri_offset = nh_private.param<float>("IntersectionPatternDevOffset", 1);
  float deviation_tri_grade = nh_private.param<float>("IntersectionPatternDevGrade", 1);
  float deviation_lin_offset = nh_private.param<float>("IntersectionPatternLineDevOffset", 1);
  float deviation_lin_grade = nh_private.param<float>("IntersectionPatternLineDevGrade", 1);

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
      // Search best suited scan triangle of a candidate map to a map triangle
      for (auto map_plane_triangle : map_plane) {
        triangle_error.clear();
        // map_plane_triangle.getEdges(map_triangle_sides);
        if (!map_plane_triangle.getEdges(map_triangle_sides)) {  // Triangle in Map
          for (auto scan_plane_triangle : scan_plane) {
            // scan_plane_triangle.getEdges(scan_triangle_sides);
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
            additional_score =
                deviation_lin_offset +
                std::max(-deviation_lin_offset, -triangle_error[min_index] / deviation_lin_grade);
          } else {
            // Triangle Evaluation
            additional_score =
                deviation_tri_offset +
                std::max(-deviation_tri_offset, -triangle_error[min_index] / deviation_lin_grade);
          }
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
      if (triangles_in_map_planes[j].size() != 0)
        score(i, j) = (int)((score(i, j) / triangles_in_map_planes[j].size()) * 100);
    }
  }
}

void PlaneMatch::filterIntersectionPoints(
    std::vector<std::vector<IntersectionCornernessPoint>> &intersection_points) {
  ros::NodeHandle nh_private("~");
  float search_radius = nh_private.param<float>("IntersectionPatternCoarseness", 1);
  float environment_max_size = nh_private.param<float>("IntersectionPatternMaxSize", 100);

  pcl::UniformSampling<pcl::PointXYZI> voxel_filter;
  voxel_filter.setRadiusSearch(search_radius);

  pcl::PassThrough<pcl::PointXYZI> passfilter;
  passfilter.setFilterLimits(-environment_max_size, environment_max_size);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointXYZI pcl_point;
  IntersectionCornernessPoint corner_point;
  int plane_nr = 0;
  for (auto plane_points : intersection_points) {
    filter_cloud->clear();
    for (auto points : plane_points) {
      pcl_point.x = points.intersection_point.x();
      pcl_point.y = points.intersection_point.y();
      pcl_point.z = points.intersection_point.z();
      pcl_point.intensity = points.cornerness;
      filter_cloud->push_back(pcl_point);
    }
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
    for (auto points : *filter_cloud) {
      corner_point = {Point(points.x, points.y, points.z), points.intensity};
      intersection_points[plane_nr].push_back(corner_point);
    }
    plane_nr++;
  }
}

void PlaneMatch::loadExampleSol(float (&transformTR)[7],
                                const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                const pcl::PointCloud<pcl::PointNormal> map_planes) {
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
  plane_assignment[0] = 0;
  plane_assignment[1] = 14;
  plane_assignment[2] = 2;
  plane_assignment[3] = 9;
  plane_assignment[4] = 14;
  plane_assignment[5] = 11;
  plane_assignment[6] = 1;
  plane_assignment[7] = 21;

  transformAverage(transformTR, plane_assignment, scan_planes, map_planes);
}

void PlaneMatch::transformAverage(float (&transformTR)[7], std::vector<int> plane_assignment,
                                  const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                  const pcl::PointCloud<pcl::PointNormal> map_planes) {
  std::cout << "Calculate Transformation via plane correspondences" << std::endl;
  // Calculate average quaternion (Map to Lidar), R_Map_to_Lidar
  Eigen::MatrixXd all_quat(4, scan_planes.size());
  Eigen::Quaterniond current_quat;
  Eigen::Vector3d map_normal;
  Eigen::Vector3d scan_normal;
  for (int plane_nr; plane_nr < scan_planes.size(); ++plane_nr) {
    scan_normal(0) = scan_planes.points[plane_nr].normal_x;
    scan_normal(1) = scan_planes.points[plane_nr].normal_y;
    scan_normal(2) = scan_planes.points[plane_nr].normal_z;
    map_normal(0) = map_planes.points[plane_assignment[plane_nr]].normal_x;
    map_normal(1) = map_planes.points[plane_assignment[plane_nr]].normal_y;
    map_normal(2) = map_planes.points[plane_assignment[plane_nr]].normal_z;
    current_quat = Eigen::Quaterniond::FromTwoVectors(scan_normal, map_normal);
    all_quat(0, plane_nr) = current_quat.w();
    all_quat(1, plane_nr) = current_quat.x();
    all_quat(2, plane_nr) = current_quat.y();
    all_quat(3, plane_nr) = current_quat.z();
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
  pcl::copyPointCloud(scan_planes, copy_scan_planes);
  Eigen::Matrix4f res_rotation = Eigen::Matrix4f::Identity();
  Eigen::Quaternionf q(transformTR[3], transformTR[4], transformTR[5], transformTR[6]);
  res_rotation.block(0, 0, 3, 3) = q.matrix();
  pcl::transformPointCloud(copy_scan_planes, copy_scan_planes, res_rotation);

  double mean_translation_x = 0;
  double mean_translation_y = 0;
  double mean_translation_z = 0;
  int nr_planes_x = 0;
  int nr_planes_y = 0;
  int nr_planes_z = 0;
  int map_index;
  for (int plane_nr = 0; plane_nr < scan_planes.size(); ++plane_nr) {
    map_index = plane_assignment[plane_nr];
    if (std::abs(map_planes.points[map_index].normal_x) > 0.5) {
      mean_translation_x =
          map_planes.points[map_index].x - copy_scan_planes.points[plane_nr].x + mean_translation_x;
      ++nr_planes_x;
    }
    if (std::abs(map_planes.points[map_index].normal_y) > 0.5) {
      mean_translation_y =
          map_planes.points[map_index].y - copy_scan_planes.points[plane_nr].y + mean_translation_y;
      ++nr_planes_y;
    }
    if (std::abs(map_planes.points[plane_nr].normal_z) > 0.5) {
      mean_translation_z =
          map_planes.points[map_index].z - copy_scan_planes.points[plane_nr].z + mean_translation_z;
      ++nr_planes_z;
    }
  }
  if (nr_planes_x != 0) {
    transformTR[0] = mean_translation_x / nr_planes_x;
  } else {
    std::cout << "Couldn't find translation x as plane with normal x is missing" << std::endl;
    transformTR[0] = 0;
  }
  if (nr_planes_y != 0) {
    transformTR[1] = mean_translation_y / nr_planes_y;
  } else {
    std::cout << "Couldn't find translation y as plane with normal y is missing" << std::endl;
    transformTR[1] = 0;
  }
  if (nr_planes_z != 0) {
    transformTR[2] = mean_translation_z / nr_planes_z;
  } else {
    std::cout << "Couldn't find translation z as plane with normal z is missing" << std::endl;
    transformTR[2] = 0;
  }
};

}  // namespace matching_algorithms
}  // namespace cad_percept