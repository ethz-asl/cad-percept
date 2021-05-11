#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/octree/octree_search.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <set>
#include <string>
#include <utility>
//#include <pcl/common/impl/intersections.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/geometry/quad_mesh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#define CGAL_USE_SCIP
#include <CGAL/SCIP_mixed_integer_program_traits.h>
typedef CGAL::SCIP_mixed_integer_program_traits<double> MIP_Solver;
typedef typename MIP_Solver::Variable Variable;
typedef typename MIP_Solver::Linear_objective Linear_objective;
typedef typename MIP_Solver::Linear_constraint Linear_constraint;

// Source:https://github.com/tttamaki/ICP-test/blob/master/src/icp3_with_normal_iterative_view.cpp
void addNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                pcl::PointCloud<pcl::Normal>::Ptr normals, int k) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  searchTree->setInputCloud(cloud);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(cloud);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(k);
  normalEstimator.compute(*normals);
}

void combineMeshes(const pcl::PolygonMesh &mesh, pcl::PolygonMesh &mesh_all) {
  // pcl::PolygonMesh::concatenate(mesh_all, mesh); ???
  // Source:
  // https://github.com/PointCloudLibrary/pcl/blob/master/common/include/pcl/PolygonMesh.h
  mesh_all.header.stamp = std::max(mesh_all.header.stamp, mesh.header.stamp);

  const auto point_offset = mesh_all.cloud.width * mesh_all.cloud.height;

  // Transform them to PointXYZ and back to PCLPointCloud (to remove redundant
  // fields)
  pcl::PointCloud<pcl::PointXYZ> mesh_all_cloud;
  pcl::PointCloud<pcl::PointXYZ> mesh_cloud;
  pcl::fromPCLPointCloud2(mesh_all.cloud, mesh_all_cloud);
  pcl::fromPCLPointCloud2(mesh.cloud, mesh_cloud);
  pcl::PCLPointCloud2 mesh_all_cloud2;
  pcl::PCLPointCloud2 mesh_cloud2;
  pcl::toPCLPointCloud2(mesh_cloud, mesh_cloud2);
  pcl::toPCLPointCloud2(mesh_all_cloud, mesh_all_cloud2);

  // Conatenate point clouds
  pcl::PCLPointCloud2 new_cloud;
  pcl::concatenatePointCloud(mesh_all_cloud2, mesh_cloud2, new_cloud);
  mesh_all.cloud = new_cloud;

  // Concatenate polygons
  std::transform(
      mesh.polygons.begin(), mesh.polygons.end(),
      std::back_inserter(mesh_all.polygons), [point_offset](auto polygon) {
        std::transform(polygon.vertices.begin(), polygon.vertices.end(),
                       polygon.vertices.begin(),
                       [point_offset](auto &point_idx) {
                         return point_idx + point_offset;
                       });
        return polygon;
      });
}

void planeFromIdx(Eigen::Vector4f &result, int idx,
                  std::vector<Eigen::Vector3d> &plane_normals,
                  std::vector<double> &plane_d) {
  Eigen::Vector3d normals = plane_normals.at(idx);
  double d = plane_d.at(idx);
  result(0) = (float)normals.x();
  result(1) = (float)normals.y();
  result(2) = (float)normals.z();
  result(3) = (float)d;
}

void computeAllPlanes(std::vector<::pcl::Vertices> &faces_model,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_model,
                      std::vector<Eigen::Vector3d> &plane_normals,
                      std::vector<double> &plane_d, std::vector<double> &area) {
  for (int i = 0; i < faces_model.size(); i++) {
    std::vector<uint32_t> vertices = faces_model.at(i).vertices;
    if (vertices.size() != 3) {
      std::cout << "Warning: Non tria element" << std::endl;
    }
    pcl::PointXYZ p1 = (*vertices_model)[vertices.at(0)];
    pcl::PointXYZ p2 = (*vertices_model)[vertices.at(1)];
    pcl::PointXYZ p3 = (*vertices_model)[vertices.at(2)];
    Eigen::Vector3d v1(p1.x, p1.y, p1.z);
    Eigen::Vector3d v2(p2.x, p2.y, p2.z);
    Eigen::Vector3d v3(p3.x, p3.y, p3.z);
    Eigen::Vector3d n = (v2 - v1).cross(v3 - v1);
    area.push_back(n.norm());

    // Normal correction up to 5 degree
    n.normalize();
    for (int i = 0; i < 3; i++) {
      if (n(i) < -0.95) {
        n(i) = -1.;
      } else if (n(i) > -0.05 && n(i) < 0.05) {
        n(i) = 0.;
      } else if (n(i) > 0.95) {
        n(i) = 1.;
      }
      n.normalize();
    }

    plane_normals.push_back(n);
    double d = -(n.x() * p1.x + n.y() * p1.y + n.z() * p1.z);
    plane_d.push_back(d);
  }
}

void flagDuplicatedPlanes(std::vector<::pcl::Vertices> &faces_model,
                          std::vector<Eigen::Vector3d> &plane_normals,
                          std::vector<double> &plane_d,
                          std::vector<double> &area,
                          std::vector<int> &duplicated_faces,
                          double min_area = 0.0) {
  for (int i = 0; i < faces_model.size(); i++) {
    if (std::find(duplicated_faces.begin(), duplicated_faces.end(), i) !=
            duplicated_faces.end() ||
        area.at(i) < min_area) {
      continue;
    }
    Eigen::Vector3d plane_normal_1 = plane_normals.at(i);
    double d_1 = plane_d.at(i);
    for (int j = i + 1; j < faces_model.size(); j++) {
      if (std::find(duplicated_faces.begin(), duplicated_faces.end(), j) !=
          duplicated_faces.end()) {
        continue;
      }
      Eigen::Vector3d plane_normal_2 = plane_normals.at(j);
      double d_2 = plane_d.at(j);

      if (std::fabs(plane_normal_1.dot(plane_normal_2)) > 0.9999 &&
          std::fabs(d_1 - d_2) < 0.003) {
        duplicated_faces.push_back(j);
      }
    }
  }
}

void processElementCloud(int idx,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr element_points,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr corners,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr corners_top_bottom,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr corners_side,
                         Eigen::Vector3d &element_normal, double &element_d) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.03);
  seg.setMaxIterations(1000);
  seg.setInputCloud(element_points);
  seg.segment(*inliers, *coefficients);

  // Compute ransac normal vector
  element_normal(0) = coefficients->values[0];
  element_normal(1) = coefficients->values[1],
  element_normal(2) = coefficients->values[2];
  element_normal.normalize();

  if (std::fabs(element_normal.z()) < 0.1) {
    element_normal.z() = 0;
    element_normal.normalize();
  }

  // TODO: ROS, get robot postion
  pcl::PointXYZ robot_position;
  if (idx == 4) {
    robot_position = pcl::PointXYZ(-9.4317884, -5.2247009, 1.0);
  } else {
    robot_position = pcl::PointXYZ(10.43749, 1.7694572, 1.0);
  }
  pcl::PointXYZ mean_point(0, 0, 0);
  for (int i = 0; i < element_points->size(); i++) {
    mean_point.x += (*element_points)[i].x;
    mean_point.y += (*element_points)[i].y;
    mean_point.z += (*element_points)[i].z;
  }
  mean_point.x /= element_points->size();
  mean_point.y /= element_points->size();
  mean_point.z /= element_points->size();
  Eigen::Vector3d shape_robot_vec(mean_point.x - robot_position.x,
                                  mean_point.y - robot_position.y,
                                  mean_point.z - robot_position.z);
  shape_robot_vec.normalize();
  // Normal direction of shape is pointing into free direction
  if (shape_robot_vec.dot(element_normal) < 0.0) {
    element_normal *= -1.;
  }
  element_d =
      -(mean_point.x * element_normal.x() + mean_point.y * element_normal.y() +
        mean_point.z * element_normal.z());

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  searchTree->setInputCloud(element_points);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(element_points);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(10);
  normalEstimator.compute(*normals);

  // Filter corner points out
  pcl::PointCloud<pcl::PointXYZI>::Ptr corners_intensity(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
  harris.setInputCloud(element_points);
  harris.setNonMaxSupression(false);
  harris.setRadius(0.2f);
  harris.setThreshold(0.2f);
  harris.setNormals(normals);
  harris.compute(*corners_intensity);

  // Remove Intensity field
  pcl::copyPointCloud(*corners_intensity, *corners);

  // Corners -> Corners top and bottom, Corners side
  if (std::fabs(element_normal.z()) < 0.1) {
    double max_z = -10000;
    double min_z = 10000;
    double max_xy = -10000;
    double min_xy = 10000;
    for (int i = 0; i < corners->size(); i++) {
      pcl::PointXYZ p = (*corners)[i];
      if (p.z > max_z) {
        max_z = p.z;
      }
      if (p.z < min_z) {
        min_z = p.z;
      }
      double dist = std::fabs(p.x) + std::fabs(p.y);
      if (dist > max_xy) {
        max_xy = dist;
      }
      if (dist < min_xy) {
        min_xy = dist;
      }
    }
    double hight = max_z - min_z;
    double width = max_xy - min_xy;
    double threshold_top = max_z - 0.15 * hight;
    double threshold_bottom = min_z + 0.15 * hight;
    double threshold_right = max_xy - 0.15 * width;
    double threshold_left = min_xy + 0.15 * width;

    for (int i = 0; i < corners->size(); i++) {
      pcl::PointXYZ p = (*corners)[i];
      if (p.z > threshold_top || p.z < threshold_bottom) {
        corners_top_bottom->push_back(p);
      }
      double dist = std::fabs(p.x) + std::fabs(p.y);
      if (dist > threshold_right || dist < threshold_left) {
        corners_side->push_back(p);
      }
    }
  } else {
    pcl::copyPointCloud(*corners, *corners_side);
    pcl::copyPointCloud(*corners, *corners_top_bottom);
  }
}

void selectMainCandidateFaces(
    std::vector<::pcl::Vertices> &faces_model,
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_model,
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners,
    Eigen::Vector3d &element_normal, double element_d,
    std::vector<int> &duplicated_faces, std::vector<int> &candidate_faces_main,
    std::vector<Eigen::Vector3d> &plane_normals, std::vector<double> &area,
    std::vector<double> &plane_d, double min_area = 0.0,
    bool plane_flipping = true) {
  std::vector<Eigen::Vector3d> flipped_planes;

  // Add planes with same similar normal and in local scope
  int nr_faces = faces_model.size();  // Break cycle!!
  for (int i = 0; i < nr_faces; i++) {
    // Ignore duplicated planes or very small areas
    if (std::find(duplicated_faces.begin(), duplicated_faces.end(), i) !=
            duplicated_faces.end() ||
        area.at(i) < min_area) {
      continue;
    }

    Eigen::Vector3d candidate_normal = plane_normals.at(i);
    double candidate_d = plane_d.at(i);

    // TODO: Consider robot position?
    if (std::fabs(element_normal.dot(candidate_normal)) > 0.99) {
      std::vector<uint32_t> vertices = faces_model.at(i).vertices;

      double min_distance = 1000;
      for (int p_idx = 0; p_idx < corners->size(); p_idx++) {
        pcl::PointXYZ p = (*corners)[p_idx];
        double error =
            std::fabs(candidate_normal.x() * p.x + candidate_normal.y() * p.y +
                      candidate_normal.z() * p.z + candidate_d);
        if (error < min_distance) {
          min_distance = error;
        }
      }

      if (min_distance <= 0.5) {
        candidate_faces_main.push_back(i);

        // Do plane flipping
        pcl::PointXYZ p1 = (*vertices_model)[vertices.at(0)];
        pcl::PointXYZ p2 = (*vertices_model)[vertices.at(1)];
        pcl::PointXYZ p3 = (*vertices_model)[vertices.at(2)];
        Eigen::Vector3d v1(p1.x, p1.y, p1.z);
        Eigen::Vector3d v2(p2.x, p2.y, p2.z);
        Eigen::Vector3d v3(p3.x, p3.y, p3.z);
        Eigen::Vector3d n = (v1 - v2).cross(v3 - v2);
        n.normalize();

        Eigen::Vector3d v1_v2 = v1 - v2;
        double length =
            std::sqrt(v1_v2.x() * v1_v2.x() + v1_v2.y() * v1_v2.y());

        if (plane_flipping && std::fabs(n.z()) < 0.1 && length >= 0.1 &&
            length <= 0.5) {
          bool already_flipped = false;
          for (int k = 0; k < flipped_planes.size(); k++) {
            Eigen::Vector3d flipping_vector = flipped_planes.at(k);
            if ((flipping_vector - (length * n)).lpNorm<Eigen::Infinity>() <
                0.01) {
              already_flipped = true;
              break;
            }
          }
          if (!already_flipped) {
            std::cout << "Do plane flipping" << std::endl;
            Eigen::Vector3d p1_pos(v1 + n * length);
            Eigen::Vector3d p2_pos(v2 + n * length);
            Eigen::Vector3d p3_pos(v3 + n * length);

            Eigen::Vector3d p1_neg(v1 - n * length);
            Eigen::Vector3d p2_neg(v2 - n * length);
            Eigen::Vector3d p3_neg(v3 - n * length);

            int vertices_size = vertices_model->size();
            vertices_model->push_back(
                pcl::PointXYZ(p1_pos.x(), p1_pos.y(), p1_pos.z()));
            vertices_model->push_back(
                pcl::PointXYZ(p2_pos.x(), p2_pos.y(), p2_pos.z()));
            vertices_model->push_back(
                pcl::PointXYZ(p3_pos.x(), p3_pos.y(), p3_pos.z()));
            pcl::Vertices vertices_shift_pos;
            vertices_shift_pos.vertices.push_back(vertices_size);
            vertices_shift_pos.vertices.push_back(vertices_size + 1);
            vertices_shift_pos.vertices.push_back(vertices_size + 2);

            vertices_model->push_back(
                pcl::PointXYZ(p1_neg.x(), p1_neg.y(), p1_neg.z()));
            vertices_model->push_back(
                pcl::PointXYZ(p2_neg.x(), p2_neg.y(), p2_neg.z()));
            vertices_model->push_back(
                pcl::PointXYZ(p3_neg.x(), p3_neg.y(), p3_neg.z()));
            pcl::Vertices vertices_shift_neg;
            vertices_shift_neg.vertices.push_back(vertices_size + 3);
            vertices_shift_neg.vertices.push_back(vertices_size + 4);
            vertices_shift_neg.vertices.push_back(vertices_size + 5);
            flipped_planes.push_back(n * length);

            candidate_faces_main.push_back(faces_model.size());
            faces_model.push_back(vertices_shift_pos);
            candidate_faces_main.push_back(faces_model.size());
            faces_model.push_back(vertices_shift_neg);

            plane_normals.push_back(n);
            plane_normals.push_back(n);
            area.push_back(n.norm());
            area.push_back(n.norm());

            double d1 =
                -(n.x() * p1_pos.x() + n.y() * p1_pos.y() + n.z() * p1_pos.z());
            double d2 =
                -(n.x() * p1_neg.x() + n.y() * p1_neg.y() + n.z() * p1_neg.z());
            plane_d.push_back(d1);
            plane_d.push_back(d2);
          }
        }
      }
    }
  }
}

void selectOrthoCandidateFaces(
    std::vector<::pcl::Vertices> &faces_model,
    std::vector<int> &duplicated_faces,
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_model,
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_side,
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_top_bottom,
    Eigen::Vector3d &element_normal,
    std::vector<Eigen::Vector3d> &plane_normals, std::vector<double> &area,
    std::vector<double> &plane_d,
    std::vector<int> &candidate_faces_ortho_horizontal,
    std::vector<int> &candidate_faces_ortho_vertical, double min_area = 0.0) {
  for (int i = 0; i < faces_model.size(); i++) {
    // Ignore duplicated planes or very small areas
    if (std::find(duplicated_faces.begin(), duplicated_faces.end(), i) !=
            duplicated_faces.end() ||
        area.at(i) < min_area) {
      continue;
    }

    Eigen::Vector3d candidate_normal = plane_normals.at(i);
    double candidate_d = plane_d.at(i);

    if (std::fabs(element_normal.dot(candidate_normal)) < 0.06) {
      std::vector<uint32_t> vertices = faces_model[i].vertices;
      double min_distance = 1000;
      if (std::fabs(candidate_normal.z()) < 0.5) {
        for (int p_idx = 0; p_idx < corners_side->size(); p_idx++) {
          pcl::PointXYZ p = (*corners_side)[p_idx];
          double error = std::fabs(candidate_normal.x() * p.x +
                                   candidate_normal.y() * p.y +
                                   candidate_normal.z() * p.z + candidate_d);
          if (error < min_distance) {
            min_distance = error;
          }
        }
        if (min_distance <= 0.4) {
          candidate_faces_ortho_vertical.push_back(i);
        }
      } else {
        for (int p_idx = 0; p_idx < corners_top_bottom->size(); p_idx++) {
          pcl::PointXYZ p = (*corners_top_bottom)[p_idx];
          double error = std::fabs(candidate_normal.x() * p.x +
                                   candidate_normal.y() * p.y +
                                   candidate_normal.z() * p.z + candidate_d);
          if (error < min_distance) {
            min_distance = error;
          }
        }
        if (min_distance <= 1.5) {
          candidate_faces_ortho_horizontal.push_back(i);
        }
      }
    }
  }
}

void computeArtificialVertices(
    std::vector<int> &candidate_faces_main,
    std::vector<int> &candidate_faces_ortho_horizontal,
    std::vector<int> &candidate_faces_ortho_vertical,
    std::vector<Eigen::Vector3d> &plane_normals, std::vector<double> &plane_d,
    Eigen::Vector3d &element_normal, double &element_d,
    pcl::PointCloud<pcl::PointXYZ>::Ptr artificial_vertices) {
  Eigen::Vector3f p1;
  Eigen::Vector4f plane_m1, plane_h1, plane_v1;
  for (int i = 0; i < candidate_faces_main.size(); i++) {
    planeFromIdx(plane_m1, candidate_faces_main.at(i), plane_normals, plane_d);

    for (int j = 0; j < candidate_faces_ortho_vertical.size(); j++) {
      planeFromIdx(plane_v1, candidate_faces_ortho_vertical.at(j),
                   plane_normals, plane_d);

      for (int k = 0; k < candidate_faces_ortho_horizontal.size(); k++) {
        planeFromIdx(plane_h1, candidate_faces_ortho_horizontal.at(k),
                     plane_normals, plane_d);

        pcl::threePlanesIntersection(plane_m1, plane_v1, plane_h1, p1);

        if (true || element_normal.x() * p1.x() + element_normal.y() * p1.y() +
                            element_normal.z() * p1.z() + element_d >=
                        -0.03) {
          artificial_vertices->push_back(pcl::PointXYZ(p1.x(), p1.y(), p1.z()));
        }
      }
    }
  }
}

void removeDuplicatedPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                            double eps = 10e-6) {
  // Source:
  // https://stackoverflow.com/questions/34481190/removing-duplicates-of-3d-points-in-a-vector-in-c/34481426
  std::sort(cloud->begin(), cloud->end(),
            [eps](pcl::PointXYZ p1, pcl::PointXYZ p2) -> bool {
              if (std::fabs(p1.x - p2.x) > eps)
                return p1.x > p2.x;
              else if (std::fabs(p1.y - p2.y) > eps)
                return p1.y > p2.y;
              else
                return p1.z > p2.z;
            });
  auto unique_end =
      std::unique(cloud->begin(), cloud->end(),
                  [eps](pcl::PointXYZ p1, pcl::PointXYZ p2) -> bool {
                    if (pcl::euclideanDistance(p1, p2) < eps) {
                      return true;
                    }
                    return false;
                  });
  cloud->erase(unique_end, cloud->end());
}

void removeDuplicatedValues(std::vector<double> &vector, double eps = 10e-6) {
  // Source:
  // https://stackoverflow.com/questions/34481190/removing-duplicates-of-3d-points-in-a-vector-in-c/34481426
  std::sort(vector.begin(), vector.end(),
            [eps](double p1, double p2) -> bool { return p1 > p2; });
  auto unique_end = std::unique(vector.begin(), vector.end(),
                                [eps](double p1, double p2) -> bool {
                                  if (std::fabs(p1 - p2) < eps) {
                                    return true;
                                  }
                                  return false;
                                });
  vector.erase(unique_end, vector.end());
}

bool reconstructSingleElement(
    int idx, pcl::PointCloud<pcl::PointXYZ>::Ptr strong_points_reconstruction,
    pcl::PointCloud<pcl::PointXYZ>::Ptr weak_points,
    std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr>
        artificial_kdtrees_vector,
    std::vector<Eigen::Vector3d> normal_detected_shapes_vector,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> element_points_vector,
    std::vector<Eigen::Vector3d> &plane_normals, std::vector<double> &area,
    std::vector<::pcl::Vertices> faces_model_only,
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_model_only,
    pcl::PolygonMesh &mesh_all) {
  // TODO: Check for other connected elements

  std::string save0 = "/home/philipp/Schreibtisch/arificial_points_" +
                      std::to_string(idx) + ".ply";
  pcl::io::savePLYFile(save0, *weak_points);

  pcl::PointCloud<pcl::PointXYZ>::Ptr upsampled_model(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PLYReader reader;
  reader.read("/home/philipp/Schreibtisch/data/CLA_MissingParts_3_5m.ply",
              *upsampled_model);

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.05f);
  octree.setInputCloud(upsampled_model);
  octree.addPointsFromInputCloud();

  // Get data
  pcl::PointCloud<pcl::PointXYZ>::Ptr element_points =
      element_points_vector.at(idx);
  Eigen::Vector3d element_normal = normal_detected_shapes_vector.at(idx);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree =
      artificial_kdtrees_vector.at(idx);

  // TODO: ROS, get robot postion
  pcl::PointXYZ robot_position(10.43749, 1.7694572, 1.0);
  pcl::PointXYZ mean_point(0, 0, 0);
  for (int i = 0; i < element_points->size(); i++) {
    mean_point.x += (*element_points)[i].x;
    mean_point.y += (*element_points)[i].y;
    mean_point.z += (*element_points)[i].z;
  }
  mean_point.x /= element_points->size();
  mean_point.y /= element_points->size();
  mean_point.z /= element_points->size();
  Eigen::Vector3d shape_robot_vec(mean_point.x - robot_position.x,
                                  mean_point.y - robot_position.y,
                                  mean_point.z - robot_position.z);
  shape_robot_vec.normalize();
  // Normal direction of shape is pointing into free direction
  if (shape_robot_vec.dot(element_normal) < 0.0) {
    // element_normal *= -1.;
    std::cout << "element_normal pointing in wrong direction!" << std::endl;
    // assert(false);
  }

  double shape_d =
      -(mean_point.x * element_normal.x() + mean_point.y * element_normal.y() +
        mean_point.z * element_normal.z());

  // Filter points (remaining vertices -> vertices on axis and free side)
  // TODO: Maybe recompute plane from vertices to reduce the tolerance?
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < weak_points->size(); i++) {
    pcl::PointXYZ p = (*weak_points)[i];
    if (p.x * element_normal.x() + p.y * element_normal.y() +
            p.z * element_normal.z() + shape_d >=
        -0.05) {
      filtered_cloud->push_back(p);
    }
  }
  // pcl::io::savePLYFile("/home/philipp/Schreibtisch/filtered_cloud1.ply",
  //                     *filtered_cloud);

  // Corp off confliction points
  // Source: https://pcl.readthedocs.io/en/latest/moment_of_inertia.html
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(filtered_cloud);
  feature_extractor.compute();
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;
  feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter(mass_center);

  major_vector.normalized();
  middle_vector.normalized();
  minor_vector.normalized();

  Eigen::Matrix<float, 3, 6> shooting_dir;
  shooting_dir.col(0) = major_vector;
  shooting_dir.col(1) = -major_vector;
  shooting_dir.col(2) = middle_vector;
  shooting_dir.col(3) = -middle_vector;
  shooting_dir.col(4) = minor_vector;
  shooting_dir.col(5) = -minor_vector;

  std::cout << shooting_dir << std::endl;
  for (int i = 0; i < 6; i++) {
    pcl::PointCloud<pcl::PointXYZ>::VectorType voxel_centers;
    Eigen::Vector3f direction = shooting_dir.col(i);
    octree.getIntersectedVoxelCenters(mass_center, direction, voxel_centers);

    // std::cout << "voxel_centers.size: " << voxel_centers.size() << std::endl;
    if (voxel_centers.size() > 0) {
      pcl::PointXYZ voxel_center = voxel_centers.at(0);
      double d_voxel_center =
          -(direction.x() * voxel_center.x + direction.y() * voxel_center.y +
            direction.z() * voxel_center.z);

      pcl::PointIndices::Ptr indices_to_remove(new pcl::PointIndices());
      for (int j = 0; j < filtered_cloud->size(); j++) {
        pcl::PointXYZ p = (*filtered_cloud)[j];
        double error = p.x * direction.x() + p.y * direction.y() +
                       p.z * direction.z() + d_voxel_center;
        if (error > 0.03) {
          indices_to_remove->indices.push_back(j);
        }
      }
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(filtered_cloud);
      extract.setIndices(indices_to_remove);
      extract.setNegative(true);
      extract.filter(*filtered_cloud);
    }
  }

  removeDuplicatedPoints(filtered_cloud);
  std::string save1 = "/home/philipp/Schreibtisch/filtered_cloud_" +
                      std::to_string(idx) + ".ply";
  pcl::io::savePLYFile(save1, *filtered_cloud);

  std::vector<pcl::PolygonMesh> mesh_proposals;

  // Structure of strong points
  pcl::PointCloud<pcl::PointXYZ>::Ptr alive_strong_points_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < strong_points_reconstruction->size(); i++) {
    pcl::PointXYZ p_f = (*strong_points_reconstruction)[i];
    for (int j = 0; j < filtered_cloud->size(); j++) {
      pcl::PointXYZ p_s = (*filtered_cloud)[j];
      double error = pcl::euclideanDistance(p_f, p_s);
      if (error < 10e-3) {
        alive_strong_points_cloud->push_back(p_s);
        break;
      }
    }
  }

  std::vector<double> strong_distances;
  for (int i = 0; i < alive_strong_points_cloud->size(); i++) {
    for (int j = 0; j < alive_strong_points_cloud->size(); j++) {
      double dist = pcl::euclideanDistance((*alive_strong_points_cloud)[i],
                                           (*alive_strong_points_cloud)[j]);
      strong_distances.push_back(dist);
    }
  }
  removeDuplicatedValues(strong_distances, 0.05);

  std::vector<double> filtered_points_distances;
  std::vector<Eigen::Vector3d> filtered_points_direction;
  std::vector<std::pair<int, int>> filtered_points_pairs;
  for (int i = 0; i < filtered_cloud->size(); i++) {
    pcl::PointXYZ p1 = (*filtered_cloud)[i];
    for (int j = i + 1; j < filtered_cloud->size(); j++) {
      pcl::PointXYZ p2 = (*filtered_cloud)[j];
      double dist = pcl::euclideanDistance(p1, p2);
      if (dist > 0.1) {
        filtered_points_distances.push_back(dist);
        filtered_points_pairs.push_back(std::make_pair(i, j));
        Eigen::Vector3d vec(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
        filtered_points_direction.push_back(vec);
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr proposal_all(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < strong_distances.size(); i++) {
    double cur_distance = strong_distances.at(i);
    for (int j = 0; j < filtered_points_distances.size(); j++) {
      if (std::fabs(cur_distance - filtered_points_distances.at(j)) < 10e-3) {
        std::pair<int, int> p = filtered_points_pairs.at(j);
        proposal_all->push_back((*filtered_cloud)[p.first]);
        proposal_all->push_back((*filtered_cloud)[p.second]);
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr resulting_points(
      new pcl::PointCloud<pcl::PointXYZ>());
  if (proposal_all->size() > 7) {
    pcl::copyPointCloud(*proposal_all, *resulting_points);
  } else {
    pcl::copyPointCloud(*filtered_cloud, *resulting_points);
  }

  // Ensure valid structure
  pcl::PolygonMesh reconstructed_mesh;
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud(resulting_points);
  chull.reconstruct(reconstructed_mesh);

  /*
  int counter = 0;
  do{
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(resulting_points);
    chull.reconstruct(reconstructed_mesh);

    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points (new
  pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(reconstructed_mesh.cloud, *hull_points);
    std::vector<pcl::Vertices> polygons = reconstructed_mesh.polygons;

    if (polygons.size() <= 12){
      break;
    }

    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (hull_points);
    feature_extractor.compute();
    std::vector <float> eccentricity;
    feature_extractor.getEccentricity(eccentricity);
    float ecc_max = eccentricity.at(0);
    int idx = 0;
    for (int i = 0; i < eccentricity.size(); i++){
      if (ecc_max < eccentricity.at(i)){
        ecc_max = eccentricity.at(i);
        idx = i;
      }
    }
    std::set<int> idx_to_remove;
    idx_to_remove.insert(idx);

    std::set<int> idx_to_remove;
    for (int i = 0; i< polygons.size(); i++){
      pcl::Vertices face = polygons.at(i);
      pcl::PointXYZ p1 = hull_points->at(face.vertices.at(0));
      pcl::PointXYZ p2 = hull_points->at(face.vertices.at(1));
      pcl::PointXYZ p3 = hull_points->at(face.vertices.at(2));

      Eigen::Vector3d e1(p1.x, p1.y, p1.z);
      Eigen::Vector3d e2(p2.x, p2.y, p2.z);
      Eigen::Vector3d e3(p3.x, p3.y, p3.z);

      Eigen::Vector3d normal = (e1 - e2).cross(e3 - e2);
      normal.normalize();

      if (normal.dot(element_normal) < 0.0){
        normal *= -1.;
      }

      //TODO -
    if (normal.dot(element_normal) > 0.05 && normal.dot(element_normal) < 0.95){
      double error_point_1 = 0;
      double error_point_2 = 0;
      double error_point_3 = 0;
      if (error_point_1 > error_point_2 && error_point_1 > error_point_3){
        idx_to_remove.insert(face.vertices.at(0));
      } else if (error_point_2 > error_point_1 && error_point_2 >
  error_point_3){ idx_to_remove.insert(face.vertices.at(1)); } else {
        idx_to_remove.insert(face.vertices.at(2));
      }
    }
    }
    if (idx_to_remove.size() == 0){
      break;
    }

    for (std::set<int>::iterator it = idx_to_remove.begin();
  it!=idx_to_remove.end(); ++it){ pcl::PointXYZ p1 = (*hull_points)[*it]; for
  (int i = 0; i < resulting_points->size(); i++){ pcl::PointXYZ p2 =
  (*resulting_points)[i]; if (pcl::euclideanDistance(p1, p2) < 10e-5){
          (*resulting_points).erase(resulting_points->begin() + i);
          break;
        }
      }
    }

    counter++;
  } while(true);
  */
  combineMeshes(reconstructed_mesh, mesh_all);
  pcl::io::savePLYFile("/home/philipp/Schreibtisch/reconstructed_mesh_all.ply",
                       mesh_all);
  return true;

  // pcl::PointCloud<pcl::Normal>::Ptr filtered_normals(new
  // pcl::PointCloud<pcl::Normal>()); addNormals(filtered_cloud,
  // filtered_normals, 3); pcl::PointCloud<pcl::PointNormal>::Ptr
  // filtered_point_normals(new pcl::PointCloud<pcl::PointNormal>());
  // pcl::concatenateFields(*filtered_cloud, *filtered_normals,
  // *filtered_point_normals);

  /*
  //Source:
  https://github.com/PointCloudLibrary/pcl/blob/master/test/filters/test_sampling.cpp
  pcl::NormalSpaceSampling<pcl::PointNormal, pcl::PointNormal> sample;
  sample.setInputCloud(filtered_point_normals);
  sample.setNormals(filtered_point_normals);
  sample.setBins(2, 2, 2);
  sample.setSample(8);

  pcl::PointCloud<pcl::PointXYZ>::Ptr candiate_cloud(new
  pcl::PointCloud<pcl::PointXYZ>()); bool found_candidate = false; int nr_points
  = filtered_cloud->size(); int counter = 0; int min_size = 1000; while(counter
  <  10e5){ std::vector<int> sample_idx; sample.setSeed(counter++);
    sample.getSample();
    sample.filter(sample_idx);

    //Validate sample_cloud
    std::vector<double> distances;
    distances.insert(distances.end(), strong_distances.begin(),
  strong_distances.end());

    for (int i = 0; i < sample_idx.size(); i++){
      for (int j = 0; j < sample_idx.size(); j++){
        double dist =
  pcl::euclideanDistance((*filtered_cloud)[sample_idx.at(i)],
  (*filtered_cloud)[sample_idx.at(j)]); distances.push_back(dist);
      }
    }
    removeDuplicatedValues(distances, 0.1);

    if (distances.size() < min_size){
      min_size = distances.size();
      std::cout << distances.size() << std::endl;
      std::cout << "Found Candidate" << std::endl;
      for (int i = 0; i < sample_idx.size(); i++) {
        candiate_cloud->push_back((*filtered_cloud)[sample_idx.at(i)]);
      }
      pcl::PolygonMesh reconstructed_mesh;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new
  pcl::PointCloud<pcl::PointXYZ>); pcl::ConvexHull<pcl::PointXYZ> chull;
      chull.setInputCloud(candiate_cloud);
      chull.reconstruct(reconstructed_mesh);
      mesh_proposals.push_back(reconstructed_mesh);

      std::string save =
  "/home/philipp/Schreibtisch/GenMeshes/gen_filtered_cloud_" +
  std::to_string(idx) + "_" + std::to_string(counter) + ".ply";
      pcl::io::savePLYFile(save, reconstructed_mesh);
    }
  }
  */
  /*
  for (int i = 0; i < mesh_proposals.size(); i++){
    std::string save = "/home/philipp/Schreibtisch/gen_filtered_cloud_" +
  std::to_string(idx) + "_" + std::to_string(i) + ".ply";
    pcl::io::savePLYFile(save, mesh_proposals.at(i));
  }
  //combineMeshes(reconstructed_mesh, mesh_all);
  //pcl::io::savePLYFile("/home/philipp/Schreibtisch/reconstructed_mesh_all.ply",
  //                     mesh_all);
  */

  // Check dimension of filtered point cloud
  double d_min = 1000;
  double d_max = -1000;
  for (int i = 0; i < filtered_cloud->size(); i++) {
    pcl::PointXYZ p = (*filtered_cloud)[i];
    double d = p.x * element_normal.x() + p.y * element_normal.y() +
               p.z * element_normal.z();
    if (d > d_max) {
      d_max = d;
    }
    if (d < d_min) {
      d_min = d;
    }
  }
  double diff_d = d_max - d_min;

  if (diff_d > 0.05) {
    std::cout << "diff_d > 0.05" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr alive_strong_points_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr strong_point_idx(new pcl::PointIndices());
    for (int i = 0; i < strong_points_reconstruction->size(); i++) {
      pcl::PointXYZ p_f = (*strong_points_reconstruction)[i];
      for (int j = 0; j < filtered_cloud->size(); j++) {
        pcl::PointXYZ p_s = (*filtered_cloud)[j];
        double error = pcl::euclideanDistance(p_f, p_s);
        if (error < 10e-3) {
          alive_strong_points_cloud->push_back(p_s);
          strong_point_idx->indices.push_back(j);
          break;
        }
      }
    }

    removeDuplicatedPoints(filtered_cloud);
    std::string save2 =
        "/home/philipp/Schreibtisch/strong_points_alive_cloud_" +
        std::to_string(idx) + ".ply";
    pcl::io::savePLYFile(save2, *alive_strong_points_cloud);

    std::vector<double> strong_points_distances;
    if (alive_strong_points_cloud->size() > 4) {
      std::cout << "alive_strong_points_cloud > 4"
                << alive_strong_points_cloud->size() << std::endl;
      // Simplified structure check
      for (int i = 0; i < alive_strong_points_cloud->size(); i++) {
        pcl::PointXYZ p1 = (*alive_strong_points_cloud)[i];
        for (int j = i + 1; j < alive_strong_points_cloud->size(); j++) {
          pcl::PointXYZ p2 = (*alive_strong_points_cloud)[j];
          double dist = pcl::euclideanDistance(p1, p2);
          strong_points_distances.push_back(dist);
        }
      }
    } else if (alive_strong_points_cloud->size() > 1) {
      std::cout << "alive_strong_points_cloud > 1"
                << alive_strong_points_cloud->size() << std::endl;
      for (int i = 0; i < alive_strong_points_cloud->size(); i++) {
        pcl::PointXYZ p1 = (*alive_strong_points_cloud)[i];
        for (int j = i + 1; j < alive_strong_points_cloud->size(); j++) {
          pcl::PointXYZ p2 = (*alive_strong_points_cloud)[j];
          double dist = pcl::euclideanDistance(p1, p2);
          strong_points_distances.push_back(dist);
        }
        for (int j = 0; j < filtered_cloud->size(); j++) {
          pcl::PointXYZ p2 = (*filtered_cloud)[j];
          Eigen::Vector3d diff(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
          if (std::fabs(diff.dot(element_normal)) < 0.01 ||
              std::fabs(diff.dot(element_normal)) > 0.99) {
            double dist = pcl::euclideanDistance(p1, p2);
            if (dist >= 0.2) {
              strong_points_distances.push_back(dist);
            }
          }
        }
      }
    } else {
      std::cout << "No strong vertices found!" << std::endl;
      for (int i = 0; i < filtered_cloud->size(); i++) {
        pcl::PointXYZ p1 = (*filtered_cloud)[i];
        for (int j = i + 1; j < filtered_cloud->size(); j++) {
          pcl::PointXYZ p2 = (*filtered_cloud)[j];
          double dist = pcl::euclideanDistance(p1, p2);
          if (dist >= 0.1) {
            strong_points_distances.push_back(dist);
          }
        }
      }
    }
    removeDuplicatedValues(strong_points_distances);

    std::vector<double> filtered_points_distances;
    std::vector<Eigen::Vector3d> filtered_points_direction;
    std::vector<std::pair<int, int>> filtered_points_pairs;
    for (int i = 0; i < filtered_cloud->size(); i++) {
      pcl::PointXYZ p1 = (*filtered_cloud)[i];
      for (int j = i + 1; j < filtered_cloud->size(); j++) {
        pcl::PointXYZ p2 = (*filtered_cloud)[j];
        double dist = pcl::euclideanDistance(p1, p2);
        if (dist > 0.1) {
          filtered_points_distances.push_back(dist);
          filtered_points_pairs.push_back(std::make_pair(i, j));
          Eigen::Vector3d vec(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
          filtered_points_direction.push_back(vec);
        }
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr proposal_all(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < strong_points_distances.size(); i++) {
      double cur_distance = strong_points_distances.at(i);
      for (int j = 0; j < filtered_points_distances.size(); j++) {
        if (std::fabs(cur_distance - filtered_points_distances.at(j)) <
            10e-12) {
          Eigen::Vector3d vec = filtered_points_direction.at(j);
          if (std::fabs(vec.dot(element_normal)) < 0.1 ||
              std::fabs(vec.dot(element_normal)) > 0.9) {
            std::pair<int, int> p = filtered_points_pairs.at(j);
            proposal_all->push_back((*filtered_cloud)[p.first]);
            proposal_all->push_back((*filtered_cloud)[p.second]);
          }
        }
      }
    }

    pcl::io::savePLYFile(
        "/home/philipp/Schreibtisch/proposal_all_with_duplicated.ply",
        *proposal_all);
    removeDuplicatedPoints(proposal_all, 0.003);
    pcl::io::savePLYFile(
        "/home/philipp/Schreibtisch/proposal_all_no_duplicated.ply",
        *proposal_all);
    // Remove outliers
    pcl::PointIndices::Ptr outlier_idx(new pcl::PointIndices());
    for (int i = 0; i < proposal_all->size(); i++) {
      pcl::PointXYZ p1 = (*proposal_all)[i];
      int counter_xy = 0;
      int counter_x = 0;
      int counter_y = 0;
      int counter_z = 0;
      for (int j = 0; j < proposal_all->size(); j++) {
        pcl::PointXYZ p2 = (*proposal_all)[j];
        if (std::fabs(p1.x - p2.x) < 0.001 && std::fabs(p1.y - p2.y) < 0.001) {
          counter_xy++;
        }
        if (std::fabs(p1.x - p2.x) < 0.001) {
          counter_x++;
        }
        if (std::fabs(p1.y - p2.y) < 0.001) {
          counter_y++;
        }
        if (std::fabs(p1.z - p2.z) < 0.001) {
          counter_z++;
        }
      }
      if (counter_x < 2 || counter_y < 2 || counter_xy < 2 || counter_z < 4) {
        outlier_idx->indices.push_back(i);
      }
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(proposal_all);
    extract.setIndices(outlier_idx);
    extract.setNegative(true);
    extract.filter(*proposal_all);
    pcl::io::savePLYFile(
        "/home/philipp/Schreibtisch/proposal_all_filtered1.ply", *proposal_all);

    pcl::PolygonMesh reconstructed_mesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(proposal_all);
    chull.reconstruct(reconstructed_mesh);

    pcl::io::savePLYFile("/home/philipp/Schreibtisch/reconstructed_mesh.ply",
                         reconstructed_mesh);

    combineMeshes(reconstructed_mesh, mesh_all);
    pcl::io::savePLYFile(
        "/home/philipp/Schreibtisch/reconstructed_mesh_all.ply", mesh_all);

    return true;
  } else {
    std::cout << "diff_d <= 0.05" << std::endl;
    // Assuming no duplicated vertices

    std::vector<std::vector<Eigen::Vector3d>> vertex_matrices;
    std::vector<int> vertices_idx;

    // Find filtered points in model
    for (int i = 0; i < filtered_cloud->size(); i++) {
      int vertex_idx;
      pcl::PointXYZ cur_point = (*filtered_cloud)[i];
      for (int j = 0; j < vertices_model_only->size(); j++) {
        pcl::PointXYZ cur_vertex = (*vertices_model_only)[j];
        double dist = pcl::euclideanDistance(cur_point, cur_vertex);
        if (dist < 10e-5) {
          vertex_idx = j;
          break;
        }
      }

      std::vector<Eigen::Vector3d> edges;
      for (int j = 0; j < faces_model_only.size(); j++) {
        std::vector<uint32_t> polygon = faces_model_only.at(j).vertices;
        pcl::PointXYZ p1 = (*vertices_model_only)[polygon.at(0)];
        pcl::PointXYZ p2 = (*vertices_model_only)[polygon.at(1)];
        pcl::PointXYZ p3 = (*vertices_model_only)[polygon.at(2)];
        Eigen::Vector3d e1(p1.x, p1.y, p1.z);
        Eigen::Vector3d e2(p2.x, p2.y, p2.z);
        Eigen::Vector3d e3(p3.x, p3.y, p3.z);
        if (polygon.at(0) == vertex_idx) {
          Eigen::Vector3d n1 = (e2 - e1);
          Eigen::Vector3d n2 = (e3 - e1);
          edges.push_back(-n1 / n1.norm());
          edges.push_back(-n2 / n2.norm());
        } else if (polygon.at(1) == vertex_idx) {
          Eigen::Vector3d n1 = (e1 - e2);
          Eigen::Vector3d n2 = (e3 - e2);
          edges.push_back(-n1 / n1.norm());
          edges.push_back(-n2 / n2.norm());
        } else if (polygon.at(2) == vertex_idx) {
          Eigen::Vector3d n1 = (e1 - e3);
          Eigen::Vector3d n2 = (e2 - e3);
          edges.push_back(-n1 / n1.norm());
          edges.push_back(-n2 / n2.norm());
        }
      }
      vertex_matrices.push_back(edges);
      vertices_idx.push_back(vertex_idx);
    }

    // Identify shifting candidates
    std::vector<int> shifting_candidates;
    std::vector<int> no_shifting_candidates;
    Eigen::Vector3d shifting_direction;
    for (int i = 0; i < vertex_matrices.size(); i++) {
      std::vector<Eigen::Vector3d> normals_from_vertex = vertex_matrices.at(i);
      for (int j = 0; j < normals_from_vertex.size(); j++) {
        if ((element_normal - normals_from_vertex.at(j)).norm() < 0.01) {
          shifting_candidates.push_back(vertices_idx.at(i));
          shifting_direction = normals_from_vertex.at(j);
        } else {
          no_shifting_candidates.push_back(vertices_idx.at(i));
        }
      }
    }

    // Find shifting candidates
    std::vector<double> distances;
    for (int i = 0; i < shifting_candidates.size(); i++) {
      pcl::PointXYZ p1 = (*vertices_model_only)[shifting_candidates.at(i)];
      for (int j = i + 1; j < shifting_candidates.size(); j++) {
        pcl::PointXYZ p2 = (*vertices_model_only)[shifting_candidates.at(j)];
        double dist = pcl::euclideanDistance(p1, p2);
        if (dist >= 0.1 && dist <= 0.5) {
          distances.push_back(dist);
        }
      }
    }

    std::cout << shifting_candidates.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr shifted_points(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr meshing_points(
        new pcl::PointCloud<pcl::PointXYZ>);
    removeDuplicatedValues(distances);
    if (false && distances.size() == 0) {
      std::cout << "distances.size() == 0" << std::endl;
      return false;
    } else if (true || distances.size() == 1) {
      for (int i = 0; i < shifting_candidates.size(); i++) {
        pcl::PointXYZ candidate =
            (*vertices_model_only)[shifting_candidates.at(i)];
        Eigen::Vector3d old_point(candidate.x, candidate.y, candidate.z);
        Eigen::Vector3d new_point =
            old_point + distances.at(0) * shifting_direction;
        meshing_points->push_back(
            pcl::PointXYZ(old_point.x(), old_point.y(), old_point.z()));
        shifted_points->push_back(
            pcl::PointXYZ(new_point.x(), new_point.y(), new_point.z()));
      }

      pcl::io::savePLYFile("/home/philipp/Schreibtisch/shifted_points.ply",
                           *meshing_points);

      if (no_shifting_candidates.size() > 0) {
        for (int i = 0; i < no_shifting_candidates.size(); i++) {
          int idx_no_shifting = no_shifting_candidates.at(i);

          for (int j = 0; j < shifting_candidates.size(); j++) {
            int idx_candidate = shifting_candidates.at(j);
          }
        }
      } else {
        std::cout << "no_shifting_candidates == 0" << std::endl;
        return false;
      }
    } else {
      std::cout << "Multiple shifts not implemented yet" << std::endl;
      // Evalute score for each shift
      return false;
    }
  }

  pcl::io::savePLYFile("/home/philipp/Schreibtisch/filtered_cloud.ply",
                       *filtered_cloud);
  return false;
}

int main() {
  double min_area = 0.0;

  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFilePLY(
      "/home/philipp/Schreibtisch/data/CLA_MissingParts_3.ply", mesh);

  pcl::PolygonMesh mesh_detected;
  pcl::io::loadPolygonFilePLY(
      "/home/philipp/Schreibtisch/Result_CGAL/mesh_all_10.ply", mesh_detected);

  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_model_only(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *vertices_model_only);
  std::vector<::pcl::Vertices> faces_model_only = mesh.polygons;

  combineMeshes(mesh_detected, mesh);

  std::vector<::pcl::Vertices> faces_model = mesh.polygons;
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_model(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *vertices_model);

  // Get all plane parameters from model
  std::vector<double> plane_d;
  std::vector<double> area;
  std::vector<Eigen::Vector3d> plane_normals;
  computeAllPlanes(faces_model, vertices_model, plane_normals, plane_d, area);

  // Flag duplicated planes
  std::vector<int> duplicated_faces;
  flagDuplicatedPlanes(faces_model, plane_normals, plane_d, area,
                       duplicated_faces, min_area);

  // Compute artificial Vertices for all shapes
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> artificial_vertices_vector;
  std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr>
      artificial_kdtrees_vector;
  std::vector<Eigen::Vector3d> normal_detected_shapes_vector;
  std::vector<double> detected_shapes_d_vector;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> element_points_vector;
  std::vector<int> shapes;
  shapes.push_back(0);
  shapes.push_back(1);
  shapes.push_back(2);
  shapes.push_back(3);
  shapes.push_back(4);
  shapes.push_back(28);
  shapes.push_back(30);
  shapes.push_back(49);
  shapes.push_back(56);

  std::string path = "/home/philipp/Schreibtisch/Result_CGAL/Meshes/";
  for (int i = 0; i < shapes.size(); i++) {
    std::string filename = "points_" + std::to_string(shapes.at(i)) + ".ply";
    std::string path_filename = path + filename;

    // Load element point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr element_points(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile<pcl::PointXYZ>(path_filename, *element_points);

    // Normal of detection
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_top_bottom(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_side(
        new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector3d element_normal;
    double element_d;
    processElementCloud(i, element_points, corners, corners_top_bottom,
                        corners_side, element_normal, element_d);
    normal_detected_shapes_vector.push_back(element_normal);
    detected_shapes_d_vector.push_back(element_d);
    element_points_vector.push_back(corners);

    // Candidates: local scope
    std::vector<int> candidate_faces_main;
    selectMainCandidateFaces(faces_model, vertices_model, corners,
                             element_normal, element_d, duplicated_faces,
                             candidate_faces_main, plane_normals, area, plane_d,
                             min_area, true);

    // Add planes with orthogonal normal and in local scope
    std::vector<int> candidate_faces_ortho_horizontal;
    std::vector<int> candidate_faces_ortho_vertical;
    selectOrthoCandidateFaces(faces_model, duplicated_faces, vertices_model,
                              corners_side, corners_top_bottom, element_normal,
                              plane_normals, area, plane_d,
                              candidate_faces_ortho_horizontal,
                              candidate_faces_ortho_vertical, min_area);

    int nr_main = candidate_faces_main.size();
    int nr_ortho_vertical = candidate_faces_ortho_vertical.size();
    int nr_ortho_horizontal = candidate_faces_ortho_horizontal.size();
    std::cout << "Number of main: " << nr_main << std::endl;
    std::cout << "Number of ortho_vertical: " << nr_ortho_vertical << std::endl;
    std::cout << "Number of ortho_horizontal: " << nr_ortho_horizontal
              << std::endl;

    // Compute all intersections directly
    pcl::PointCloud<pcl::PointXYZ>::Ptr artificial_vertices(
        new pcl::PointCloud<pcl::PointXYZ>);
    computeArtificialVertices(
        candidate_faces_main, candidate_faces_ortho_horizontal,
        candidate_faces_ortho_vertical, plane_normals, plane_d, element_normal,
        element_d, artificial_vertices);

    artificial_vertices_vector.push_back(artificial_vertices);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    kd_tree->setInputCloud(artificial_vertices);
    artificial_kdtrees_vector.push_back(kd_tree);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr all_verticies(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < artificial_vertices_vector.size(); i++) {
    (*all_verticies) += *(artificial_vertices_vector.at(i));
  }
  pcl::io::savePLYFile("/home/philipp/Schreibtisch/all_artificial_vertices.ply",
                       *all_verticies);

  // Split point cloud in three parts:
  // 1. Points which are matching the vertices in the building model (strong
  // points)
  // 2. Points computed multiple times (weak points)
  // 3. Remaining points (backup)
  pcl::PointCloud<pcl::PointXYZ>::Ptr strong_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr weak_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr backup_points(
      new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<int> nn_indices{1};
  std::vector<float> nn_dists{1};

  for (int i = 0; i < vertices_model_only->size(); i++) {
    pcl::PointXYZ p_model = (*vertices_model_only)[i];
    for (int j = 0; j < artificial_kdtrees_vector.size(); j++) {
      artificial_kdtrees_vector.at(j)->nearestKSearch(p_model, 1, nn_indices,
                                                      nn_dists);
      if (nn_dists[0] < 10e-3) {
        strong_points->push_back(p_model);
        break;
      }
    }
  }

  std::vector<int> duplicated_vertices;
  for (int i = 0; i < all_verticies->size(); i++) {
    pcl::PointXYZ p_1 = (*all_verticies)[i];
    for (int j = i + 1; j < all_verticies->size(); j++) {
      pcl::PointXYZ p_2 = (*all_verticies)[j];
      double dist = std::sqrt((p_1.x - p_2.x) * (p_1.x - p_2.x) +
                              (p_1.y - p_2.y) * (p_1.y - p_2.y) +
                              (p_1.z - p_2.z) * (p_1.z - p_2.z));
      if (dist < 10e-3) {
        weak_points->push_back(p_1);
        break;
      }
    }
  }
  // Remove duplicates
  removeDuplicatedPoints(strong_points);
  removeDuplicatedPoints(weak_points);

  pcl::io::savePLYFile("/home/philipp/Schreibtisch/strong_points.ply",
                       *strong_points);
  pcl::io::savePLYFile("/home/philipp/Schreibtisch/weak_points.ply",
                       *weak_points);

  // reconstruction
  // Starting with elements element with the highest support of existing
  // vertices
  int max_idx = 0;
  int max_value = 0;
  pcl::PolygonMesh mesh_all;
  std::vector<int> done_shapes;
  pcl::PointCloud<pcl::PointXYZ>::Ptr strong_points_reconstruction;
  do {
    for (int i = 0; i < artificial_kdtrees_vector.size(); i++) {
      if (std::find(done_shapes.begin(), done_shapes.end(), i) !=
          done_shapes.end()) {
        continue;
      }

      int counter_model_vertices = 0;
      pcl::PointCloud<pcl::PointXYZ>::Ptr strong_points_reconstruction_temp(
          new pcl::PointCloud<pcl::PointXYZ>);
      for (int j = 0; j < strong_points->size(); j++) {
        pcl::PointXYZ p = (*strong_points)[j];
        artificial_kdtrees_vector.at(i)->nearestKSearch(p, 1, nn_indices,
                                                        nn_dists);
        if (nn_dists[0] < 10e-4) {
          counter_model_vertices++;
          strong_points_reconstruction_temp->push_back(p);
        }
      }
      if (max_value <= counter_model_vertices) {
        max_value = counter_model_vertices;
        max_idx = i;
        strong_points_reconstruction = strong_points_reconstruction_temp;
      }
    }

    // Reconstruct i and remove it
    std::cout << "Element to reconstruct: " << max_idx
              << " number of strong points: " << max_value << std::endl;
    reconstructSingleElement(
        max_idx, strong_points_reconstruction,
        artificial_vertices_vector.at(max_idx), artificial_kdtrees_vector,
        normal_detected_shapes_vector, element_points_vector, plane_normals,
        area, faces_model_only, vertices_model_only, mesh_all);

    pcl::PointCloud<pcl::PointXYZ>::Ptr new_strong_points(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(mesh_all.cloud, *new_strong_points);
    *strong_points += *new_strong_points;
    done_shapes.push_back(max_idx);
    max_idx = 0;
    max_value = 0;

  } while (done_shapes.size() < artificial_vertices_vector.size());

  return 0;
}