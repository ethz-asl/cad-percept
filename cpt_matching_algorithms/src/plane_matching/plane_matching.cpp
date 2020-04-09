#include "plane_matching/plane_matching.h"

using namespace cad_percept::cgal;

namespace cad_percept {
namespace matching_algorithms {

void PlaneMatch::IntersectionPatternMatcher(float (&transformTR)[7],
                                            const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                            const pcl::PointCloud<pcl::PointNormal> map_planes) {
  float threshold_horizontal = 0.8;
  float parallel_threshold = 0.5;
  std::cout << "////  Intersection Pattern Matcher Started  ////" << std::endl;

  std::vector<std::vector<Eigen::Vector3d>> scan_intersection_points;
  std::vector<std::vector<std::array<int, 3>>> used_scan_planes;
  std::vector<std::vector<Eigen::Vector3d>> map_intersection_points;
  std::vector<std::vector<std::array<int, 3>>> used_map_planes;

  // Find intersection points of intersection line of two planes with one other plane
  //  getPlaneIntersectionPoints(scan_intersection_points, used_scan_planes, parallel_threshold,
  //                             scan_planes);
  getprojPlaneIntersectionPoints(map_intersection_points, used_map_planes, parallel_threshold,
                                 map_planes);

  int plane = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ points;
  for (auto map_plane : map_intersection_points[plane]) {
    // std::cout << map_plane(0) << " " << map_plane(1) << " " << map_plane(2) << std::endl;
    points.x = map_plane(0);
    points.y = map_plane(1);
    points.z = map_plane(2);
    test_cloud->push_back(points);
  }
  float search_radius = 0.2;
  CloudFilter::filterVoxelCentroid(search_radius, *test_cloud);

  ros::NodeHandle nh;
  ros::Publisher test_pub = nh.advertise<sensor_msgs::PointCloud2>("test_extracted", 1, true);
  std::string tf_map_frame = "/map";
  sensor_msgs::PointCloud2 segmentation_mesg;
  test_cloud->header.frame_id = tf_map_frame;
  pcl::toROSMsg(*test_cloud, segmentation_mesg);
  test_pub.publish(segmentation_mesg);

  ros::spin();

  // getCornerDescriptors();
  // getCornerDescriptors();

  // getCornerMatchScore();
  // getCornerMatchScore();
}

void PlaneMatch::getprojPlaneIntersectionPoints(
    std::vector<std::vector<Eigen::Vector3d>> &tot_plane_intersections,
    std::vector<std::vector<std::array<int, 3>>> &used_planes, float parallel_threshold,
    const pcl::PointCloud<pcl::PointNormal> planes) {
  Plane cgal_plane;
  Plane cgal_pair_plane;
  Line intersection_line;

  Plane cgal_candidate_plane;
  Point intersection_point;
  std::array<int, 3> used_plane;

  tot_plane_intersections.clear();
  tot_plane_intersections =
      std::vector<std::vector<Eigen::Vector3d>>(planes.size(), std::vector<Eigen::Vector3d>(0));
  used_planes.clear();
  used_planes = std::vector<std::vector<std::array<int, 3>>>(planes.size(),
                                                             std::vector<std::array<int, 3>>(0));

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
        if (cgal_plane.orthogonal_vector() * cgal_pair_plane.orthogonal_vector() >
            parallel_threshold)
          continue;

        result_intersection_line = CGAL::intersection(cgal_plane, cgal_pair_plane);
        // Skip plane pair if there is no intersection_line
        if (!CGAL::assign(intersection_line, result_intersection_line)) continue;

        result_intersection_point = CGAL::intersection(cgal_candidate_plane, intersection_line);
        // Skip intersection plane if there is no intersection with candidate plane
        if (!CGAL::assign(intersection_point, result_intersection_point)) continue;

        tot_plane_intersections[candidate_nr].push_back(
            Eigen::Vector3d((double)(intersection_point.x()), (double)(intersection_point.y()),
                            (double)(intersection_point.z())));
        used_plane = {candidate_nr, plane_nr, plane_pair_nr};
        used_planes[candidate_nr].push_back(used_plane);
      }
    }
    ++candidate_nr;
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