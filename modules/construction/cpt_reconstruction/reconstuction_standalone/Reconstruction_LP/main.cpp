#include <cmath>
#include <iostream>
#include <utility>

#include <Eigen/Core>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
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

void combineMeshes(const pcl::PolygonMesh &mesh, pcl::PolygonMesh &mesh_all) {
  // pcl::PolygonMesh::concatenate(mesh_all, mesh); ???
  // Source:
  // https://github.com/PointCloudLibrary/pcl/blob/master/common/include/pcl/PolygonMesh.h
  mesh_all.header.stamp = std::max(mesh_all.header.stamp, mesh.header.stamp);

  const auto point_offset = mesh_all.cloud.width * mesh_all.cloud.height;

  pcl::PointCloud<pcl::PointXYZ> mesh_all_cloud;
  pcl::PointCloud<pcl::PointXYZ> mesh_cloud;
  pcl::fromPCLPointCloud2(mesh_all.cloud, mesh_all_cloud);
  pcl::fromPCLPointCloud2(mesh.cloud, mesh_cloud);

  pcl::PCLPointCloud2 mesh_all_cloud2;
  pcl::PCLPointCloud2 mesh_cloud2;
  pcl::toPCLPointCloud2(mesh_cloud, mesh_cloud2);
  pcl::toPCLPointCloud2(mesh_all_cloud, mesh_all_cloud2);

  pcl::PCLPointCloud2 new_cloud;
  pcl::concatenatePointCloud(mesh_all_cloud2, mesh_cloud2, new_cloud);
  mesh_all.cloud = new_cloud;

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
    n.normalize();

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

void processElementCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr element_points,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr corners,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr corners_top_bottom,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr corners_side,
                         Eigen::Vector3d &element_normal) {
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

  element_normal(0) = coefficients->values[0];
  element_normal(1) = coefficients->values[1],
  element_normal(2) = coefficients->values[2];
  element_normal.normalize();

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  searchTree->setInputCloud(element_points);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(element_points);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(10);
  normalEstimator.compute(*normals);

  pcl::PointCloud<pcl::PointXYZI>::Ptr corners_intensity(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
  harris.setInputCloud(element_points);
  harris.setNonMaxSupression(false);
  harris.setRadius(0.2f);
  harris.setThreshold(0.2f);
  harris.setNormals(normals);
  harris.compute(*corners_intensity);

  pcl::copyPointCloud(*corners_intensity, *corners);

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
    Eigen::Vector3d &element_normal, std::vector<int> &duplicated_faces,
    std::vector<int> &candidate_faces_main,
    std::vector<Eigen::Vector3d> &plane_normals, std::vector<double> &area,
    std::vector<double> &plane_d, double min_area = 0.0,
    bool plane_flipping = true) {
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

    if (std::fabs(element_normal.dot(candidate_normal)) < 0.1) {
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
        if (min_distance <= 0.2) {
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
        artificial_vertices->push_back(pcl::PointXYZ(p1.x(), p1.y(), p1.z()));
      }
    }
  }
}

int main() {
  double min_area = 0.0;

  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFilePLY(
      "/home/philipp/Schreibtisch/data/CLA_MissingParts_1.ply", mesh);

  pcl::PolygonMesh mesh_detected;
  pcl::io::loadPolygonFilePLY(
      "/home/philipp/Schreibtisch/Result_CGAL/mesh_all_10.ply", mesh_detected);

  combineMeshes(mesh_detected, mesh);

  // Load element point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr element_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPLYFile<pcl::PointXYZ>(
      "/home/philipp/Schreibtisch/Result_CGAL/Meshes/points_28.ply",
      *element_points);

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

  // Normal of detection
  pcl::PointCloud<pcl::PointXYZ>::Ptr corners(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr corners_top_bottom(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr corners_side(
      new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Vector3d element_normal;
  processElementCloud(element_points, corners, corners_top_bottom, corners_side,
                      element_normal);

  pcl::io::savePLYFile("/home/philipp/Schreibtisch/corners.ply", *corners);
  pcl::io::savePLYFile("/home/philipp/Schreibtisch/corners_top_bottom.ply",
                       *corners_top_bottom);
  pcl::io::savePLYFile("/home/philipp/Schreibtisch/corners_side.ply",
                       *corners_side);

  // Candidates: local scope
  std::vector<int> candidate_faces_main;
  selectMainCandidateFaces(faces_model, vertices_model, corners, element_normal,
                           duplicated_faces, candidate_faces_main,
                           plane_normals, area, plane_d, min_area, false);

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
  computeArtificialVertices(candidate_faces_main,
                            candidate_faces_ortho_horizontal,
                            candidate_faces_ortho_vertical, plane_normals,
                            plane_d, artificial_vertices);

  pcl::io::savePLYFile("/home/philipp/Schreibtisch/reconstructed.ply",
                       *artificial_vertices);

  return 0;
}