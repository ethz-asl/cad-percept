#include "plane_matching/plane_matching.h"

using namespace cad_percept::cgal;

namespace cad_percept {
namespace matching_algorithms {

float PlaneMatch::prrus(float (&transformTR)[7],
                        const pcl::PointCloud<pcl::PointNormal> scan_planes,
                        BoundedPlanes map_planes) {
  std::cout << "////////////////////////////////////////////////" << std::endl;
  std::cout << "////              PRRUS Started             ////" << std::endl;
  std::cout << "////////////////////////////////////////////////" << std::endl;

  ros::NodeHandle nh_private("~");
  float translation_penalty = nh_private.param<float>("LineSegmentRansacTranErrorPenalty", 20);

  pcl::PointCloud<pcl::PointNormal> map_planes_pc = map_planes.getPlaneCentroidsAndNormals();

  // Find groups of orthogonal planes in map
  std::vector<Eigen::Vector3i> orthogonal_groups_in_map;
  Eigen::Vector3f map_plane_normal[3];
  Eigen::Vector3f cross_product;
  for (int i = 0; i < ((int)map_planes_pc.size() - 2); ++i) {
    map_plane_normal[0] =
        Eigen::Vector3f(map_planes_pc.points[i].normal_x, map_planes_pc.points[i].normal_y,
                        map_planes_pc.points[i].normal_z);
    for (int j = i + 1; j < ((int)map_planes_pc.size() - 1); ++j) {
      map_plane_normal[1] =
          Eigen::Vector3f(map_planes_pc.points[j].normal_x, map_planes_pc.points[j].normal_y,
                          map_planes_pc.points[j].normal_z);

      // Continue if normals are collinear
      cross_product = map_plane_normal[0].cross(map_plane_normal[1]);
      if (cross_product.norm() < 0.8) continue;
      for (int k = j + 1; k < map_planes_pc.size(); ++k) {
        map_plane_normal[2] =
            Eigen::Vector3f(map_planes_pc.points[k].normal_x, map_planes_pc.points[k].normal_y,
                            map_planes_pc.points[k].normal_z);
        if (std::abs(cross_product.dot(map_plane_normal[2])) > 0.8) {
          // If orthogonal set, save combination
          orthogonal_groups_in_map.push_back(Eigen::Vector3i(i, j, k));
        }
      }
    }
  }
  if (orthogonal_groups_in_map.size() == 0) {
    std::cout
        << "Can not find transformation, as no group of orthogonal planes in map could be found"
        << std::endl;
  } else {
    std::cout << "Found " << orthogonal_groups_in_map.size()
              << " groups of orthogonal planes in map" << std::endl;
  }

  // Find groups of orthogonal planes in scan
  std::vector<Eigen::Vector3i> orthogonal_groups_in_scan;
  Eigen::Vector3f scan_plane_normal[3];
  for (int i = 0; i < ((int)scan_planes.size() - 2); ++i) {
    scan_plane_normal[0] =
        Eigen::Vector3f(scan_planes.points[i].normal_x, scan_planes.points[i].normal_y,
                        scan_planes.points[i].normal_z);
    for (int j = i + 1; j < ((int)scan_planes.size() - 1); ++j) {
      scan_plane_normal[1] =
          Eigen::Vector3f(scan_planes.points[j].normal_x, scan_planes.points[j].normal_y,
                          scan_planes.points[j].normal_z);

      // Continue if normals are collinear
      cross_product = scan_plane_normal[0].cross(scan_plane_normal[1]);
      if (cross_product.norm() < 0.8) continue;
      for (int k = j + 1; k < scan_planes.size(); ++k) {
        scan_plane_normal[2] =
            Eigen::Vector3f(scan_planes.points[k].normal_x, scan_planes.points[k].normal_y,
                            scan_planes.points[k].normal_z);
        if (std::abs(cross_product.dot(scan_plane_normal[2])) > 0.8) {
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
  if (orthogonal_groups_in_scan.size() == 0) {
    std::cout
        << "Can not find transformation, as no group of orthogonal planes in scan could be found"
        << std::endl;
  } else {
    std::cout << "Found " << orthogonal_groups_in_scan.size()
              << " groups of orthogonal planes in scan" << std::endl;
  }

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

  if (assignments.size() == 0) {
    std::cout << "Can not find transformation, as no consistent rotation could be found"
              << std::endl;
  } else {
    std::cout << "Found " << assignments.size() << " possible assignments" << std::endl;
  }

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
    if (error < min_error || min_error < 0) {
      for (int i = 0; i < 7; ++i) {
        transformTR[i] = actual_transformation[i];
      }
      min_error = error;
      std::cout << "error: " << min_error << std::endl;
    }
    ++assignment_nr;
  }
  std::cout << "final error: " << min_error << std::endl;
  return min_error;
};

void PlaneMatch::getTranslationError(Eigen::Matrix<int, 2, Eigen::Dynamic> assignment,
                                     float (&actual_transformation)[7], float &transl_error,
                                     Eigen::Quaternionf rotation,
                                     const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                     BoundedPlanes map_planes, float translation_penalty) {
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

  // Calculate translation
  Eigen::Vector3f translation(0, 0, 0);
  for (int plane_nr = 0; plane_nr < copy_scan_planes.size(); ++plane_nr) {
    translation +=
        (Eigen::Vector3f(reduced_map_planes.points[plane_nr].x,
                         reduced_map_planes.points[plane_nr].y,
                         reduced_map_planes.points[plane_nr].z) -
         Eigen::Vector3f(copy_scan_planes.points[plane_nr].x, copy_scan_planes.points[plane_nr].y,
                         copy_scan_planes.points[plane_nr].z))
            .dot(Eigen::Vector3f(reduced_map_planes.points[plane_nr].normal_x,
                                 reduced_map_planes.points[plane_nr].normal_y,
                                 reduced_map_planes.points[plane_nr].normal_z)) *
        Eigen::Vector3f(reduced_map_planes.points[plane_nr].normal_x,
                        reduced_map_planes.points[plane_nr].normal_y,
                        reduced_map_planes.points[plane_nr].normal_z);
  }
  actual_transformation[0] = translation[0];
  actual_transformation[1] = translation[1];
  actual_transformation[2] = translation[2];

  // Get transformation matrix
  Eigen::Matrix4f actual_transform = Eigen::Matrix4f::Identity();
  actual_transform.block(0, 3, 3, 1) = translation;
  actual_transform.block(0, 0, 3, 3) = rotation.matrix();

  copy_scan_planes.clear();
  pcl::transformPointCloudWithNormals(scan_planes, copy_scan_planes, actual_transform);

  // Get assignment error of found transformation
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

}  // namespace matching_algorithms
}  // namespace cad_percept