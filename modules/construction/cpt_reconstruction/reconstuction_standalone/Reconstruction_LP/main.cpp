#include <iostream>
#include <utility>
#include <cmath>

#include <Eigen/Core>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLHeader.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
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
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/geometry/quad_mesh.h>
#include <pcl/visualization/pcl_visualizer.h>

#define CGAL_USE_SCIP
#include <CGAL/SCIP_mixed_integer_program_traits.h>
typedef CGAL::SCIP_mixed_integer_program_traits<double> MIP_Solver;
typedef typename MIP_Solver::Variable                        Variable;
typedef typename MIP_Solver::Linear_objective        Linear_objective;
typedef typename MIP_Solver::Linear_constraint        Linear_constraint;


void combineMeshes(const pcl::PolygonMesh &mesh,
                   pcl::PolygonMesh &mesh_all) {
  // pcl::PolygonMesh::concatenate(mesh_all, mesh); ???
  // Source:
  // https://github.com/PointCloudLibrary/pcl/blob/master/common/include/pcl/PolygonMesh.h
  mesh_all.header.stamp = std::max(mesh_all.header.stamp, mesh.header.stamp);

  const auto point_offset = mesh_all.cloud.width * mesh_all.cloud.height;

  pcl::PCLPointCloud2 new_cloud;
  pcl::concatenatePointCloud(mesh_all.cloud, mesh.cloud, new_cloud);
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

int main() {
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFilePLY("/home/philipp/Schreibtisch/data/cla_c_vf_20150815_2020_demo.ply", mesh);

  std::vector<::pcl::Vertices> faces = mesh.polygons;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *points);

  std::cout << "Number faces: " << faces.size() << std::endl;

  std::vector<int> blocking_idx;
  std::vector<std::pair<int, int>> quads;

  //Group elements sharing the a half edge and have the same surface normal
  for (int i = 0; i < faces.size(); i++){
    if (std::find(blocking_idx.begin(), blocking_idx.end(), i) !=
            blocking_idx.end()) {
      continue;
    }
    std::vector<uint32_t> vertices_1 = faces[i].vertices;

    pcl::PointXYZ p1_1 = (*points)[vertices_1.at(0)];
    pcl::PointXYZ p1_2 = (*points)[vertices_1.at(1)];
    pcl::PointXYZ p1_3 = (*points)[vertices_1.at(2)];
    Eigen::Vector3d v1_1(p1_1.x, p1_1.y, p1_1.z);
    Eigen::Vector3d v1_2(p1_2.x, p1_2.y, p1_2.z);
    Eigen::Vector3d v1_3(p1_3.x, p1_3.y, p1_3.z);
    double l1_1 = (v1_1 - v1_2).norm();
    double l1_2 = (v1_2 - v1_3).norm();
    double l1_3 = (v1_3 - v1_1).norm();
    double l1_max = (l1_1 > l1_2) ? l1_1 : l1_2;
    l1_max = (l1_3 > l1_max) ? l1_3 : l1_max;

    for (int j = i + 1; j < faces.size(); j++){
      if (std::find(blocking_idx.begin(), blocking_idx.end(), j) !=
          blocking_idx.end()) {
        continue;
      }
      std::vector<uint32_t> vertices_2 = faces[j].vertices;
      int v_matches = 0;

      // Check if two points matches exactly
      //TODO: Assumption that vertices do not exist twice!
      for (int n = 0; n < 3; n++){
        for (int m = 0; m < 3; m++){
          if (vertices_1.at(n) == vertices_2.at(m)){
            v_matches++;
            break;
          }
        }
      }

      if(v_matches == 2){
        pcl::PointXYZ p2_1 = (*points)[vertices_2.at(0)];
        pcl::PointXYZ p2_2 = (*points)[vertices_2.at(1)];
        pcl::PointXYZ p2_3 = (*points)[vertices_2.at(2)];
        Eigen::Vector3d v2_1(p2_1.x, p2_1.y, p2_1.z);
        Eigen::Vector3d v2_2(p2_2.x, p2_2.y, p2_2.z);
        Eigen::Vector3d v2_3(p2_3.x, p2_3.y, p2_3.z);
        double l2_1 = (v2_1 - v2_2).norm();
        double l2_2 = (v2_2 - v2_3).norm();
        double l2_3 = (v2_3 - v2_1).norm();
        double l2_max = (l2_1 > l2_2) ? l2_1 : l2_2;
        l2_max = (l2_3 > l2_max) ? l2_3 : l2_max;

        //Matching half edge
        if(std::abs(l1_max - l2_max) < 10e-15){
          // Check if normal is the same
          Eigen::Vector3d n1 = (v1_1- v1_2).cross(v1_3 - v1_2);
          Eigen::Vector3d n2 = (v2_1- v2_2).cross(v2_3 - v2_2);
          n1.normalize();
          n2.normalize();

          //Check for same normal
          if ( (n1 - n2).lpNorm<Eigen::Infinity>() < 10e-15 || (n1 + n2).lpNorm<Eigen::Infinity>() < 10e-15){
            //Match found
            blocking_idx.push_back(i);
            blocking_idx.push_back(j);
            quads.push_back(std::make_pair(i, j));
            break;
          }
        }
      }
    }
  }

  std::cout << "Size of pairs: " << quads.size() << std::endl;
  pcl::PolygonMesh mesh_all;
  for (int i = 0; i < quads.size(); i++){
    std::pair<int,int> p = quads.at(i);

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<uint32_t> vertices_1 = faces[p.first].vertices;
    std::vector<uint32_t> vertices_2 = faces[p.second].vertices;

    //Start at a correct edge!!
    pcl::PointXYZ p1 = (*points)[vertices_1.at(0)];
    pcl::PointXYZ p2 = (*points)[vertices_1.at(1)];
    pcl::PointXYZ p3 = (*points)[vertices_1.at(2)];
    Eigen::Vector3d v1(p1.x, p1.y, p1.z);
    Eigen::Vector3d v2(p2.x, p2.y, p2.z);
    Eigen::Vector3d v3(p3.x, p3.y, p3.z);
    double l1 = (v1 - v2).norm();
    double l2 = (v2 - v3).norm();
    double l3 = (v3 - v1).norm();

    int start_point;
    if (l1 > l2 && l1 > l3){
      start_point = 1;
    } else if (l2 > l1 && l2 > l3){
      start_point = 2;
    } else if (l3 > l1 && l3 > l1){
      start_point = 3;
    } else{
      std::cout << "Error in starting point" << std::endl;
    }


    for (int j = 0; j < 3; j++){
      int idx = vertices_1.at(j);
      plane->push_back((*points)[idx]);
    }

    for (int j = 0; j < 3; j++){
      int idx = vertices_2.at(j);
      if (std::find(vertices_1.begin(), vertices_1.end(), idx) == vertices_1.end()) {
        plane->push_back((*points)[idx]);
      }
    }

    pcl::PolygonMesh mesh;
    pcl::PCLPointCloud2 temp_cloud;
    pcl::toPCLPointCloud2(*plane, temp_cloud);
    std::vector<pcl::Vertices> polygons_new;
    pcl::Vertices vertices_new;

    if (start_point == 1){
      vertices_new.vertices.push_back(0);
      vertices_new.vertices.push_back(3);
      vertices_new.vertices.push_back(1);
      vertices_new.vertices.push_back(2);
    } else if (start_point == 2){
      vertices_new.vertices.push_back(0);
      vertices_new.vertices.push_back(1);
      vertices_new.vertices.push_back(3);
      vertices_new.vertices.push_back(2);
    } else if (start_point == 3){
      vertices_new.vertices.push_back(0);
      vertices_new.vertices.push_back(1);
      vertices_new.vertices.push_back(2);
      vertices_new.vertices.push_back(3);
    }
    polygons_new.push_back(vertices_new);
    mesh.polygons = polygons_new;
    mesh.cloud = temp_cloud;

    combineMeshes(mesh, mesh_all);
  }
  pcl::io::savePLYFile("/home/philipp/Schreibtisch/mesh_all2.ply",mesh_all);

  //Load element point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr element_points (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPLYFile<pcl::PointXYZ> ("/home/philipp/Schreibtisch/Result_CGAL/Meshes/points_28.ply", *element_points);

  std::vector<::pcl::Vertices> faces_model = mesh.polygons;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_model(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *points_model);

  //TODO: Add flipped planes as well

  //Get all plane parameters from model
  std::vector<double> plane_d;
  std::vector<double> area;
  std::vector<Eigen::Vector3d> plane_normals;
  for (int i = 0; i < faces_model.size(); i++){
    std::vector<uint32_t> vertices = faces_model[i].vertices;
    pcl::PointXYZ p1 = (*points)[vertices.at(0)];
    pcl::PointXYZ p2 = (*points)[vertices.at(1)];
    pcl::PointXYZ p3 = (*points)[vertices.at(2)];
    Eigen::Vector3d v1(p1.x, p1.y, p1.z);
    Eigen::Vector3d v2(p2.x, p2.y, p2.z);
    Eigen::Vector3d v3(p3.x, p3.y, p3.z);
    Eigen::Vector3d n = (v1- v2).cross(v3 - v2);
    area.push_back(n.norm());
    n.normalize();

    plane_normals.push_back(n);
    double d = -(n.x() * p1.x + n.y() * p1.y + n.z() * p1.z);
    plane_d.push_back(d);
  }

  //Flag duplicated planes
  std::vector<int> duplicated_faces;
  for (int i = 0; i < faces_model.size(); i++){
    if (std::find(duplicated_faces.begin(), duplicated_faces.end(), i) !=
            duplicated_faces.end()) {
      continue;
    }
    Eigen::Vector3d plane_normal_1 = plane_normals.at(i);
    double d_1 = plane_d.at(i);
    Eigen::Vector4d plane_1(plane_normal_1.x(), plane_normal_1.y(), plane_normal_1.z(), d_1);
    for (int j = i + 1; j < faces_model.size(); j++){
      if (std::find(duplicated_faces.begin(), duplicated_faces.end(), j) !=
          duplicated_faces.end()) {
        continue;
      }
      Eigen::Vector3d plane_normal_2 = plane_normals.at(j);
      double d_2 = plane_d.at(j);
      Eigen::Vector4d plane_2(plane_normal_2.x(), plane_normal_2.y(), plane_normal_2.z(), d_2);

      if ( (plane_1 - plane_2).lpNorm<Eigen::Infinity>() < 0.1 || area.at(j) < 0.1){
        duplicated_faces.push_back(j);
      }
    }
  }

  //Create pairs of planes
  //TODO: Local scope
  std::vector<std::pair<int, int>> face_pairs;
  for (int i = 0; i < faces_model.size(); i++){
    if (std::find(duplicated_faces.begin(), duplicated_faces.end(), i) !=
        duplicated_faces.end()) {
      continue;
    }
    Eigen::Vector3d n1 = plane_normals.at(i);
    for (int j = i + 1; j < faces_model.size(); j++){
      if (std::find(duplicated_faces.begin(), duplicated_faces.end(), j) !=
          duplicated_faces.end()) {
        continue;
      }
      Eigen::Vector3d n2 = plane_normals.at(j);
      if ( std::abs(n1.dot(n2)) < 0.001){
        face_pairs.push_back(std::make_pair(i, j));
      }
    }
  }

  std::cout << "Number of pairs: " << face_pairs.size() << std::endl;
  MIP_Solver solver;
  std::vector<Variable*> variables = solver.create_variables(face_pairs.size());

  //Set constraints
  Linear_constraint* c1 = solver.create_constraint(3, 3, "c1");
  for (int i = 0; i < face_pairs.size(); i++){
    variables.at(i)->set_variable_type(Variable::BINARY);
    c1->add_coefficient(variables.at(i), 1);
  }

  //Set objective
  Linear_objective * obj = solver.create_objective(Linear_objective::MAXIMIZE);
  for (int i = 0; i < face_pairs.size(); i++){
    double l1 = 0;

    // L1 Coverage
    int idx_1 = face_pairs.at(i).first;
    int idx_2 = face_pairs.at(i).second;

    Eigen::Vector3d plane_normal_1 = plane_normals.at(idx_1);
    double a_1 = plane_normal_1.x();
    double b_1 = plane_normal_1.y();
    double c_1 = plane_normal_1.z();
    double d_1 = plane_d.at(idx_1);

    Eigen::Vector3d plane_normal_2 = plane_normals.at(idx_2);
    double a_2 = plane_normal_2.x();
    double b_2 = plane_normal_2.y();
    double c_2 = plane_normal_2.z();
    double d_2 = plane_d.at(idx_2);

    for (int n = 0; n < element_points->size(); n++){
      pcl::PointXYZ p = (*element_points)[n];
      double error1 = a_1 * p.x + b_1 * p.y + c_1 * p.z + d_1;
      double error2 = a_2 * p.x + b_2 * p.y + c_2 * p.z + d_2;
      //double error = error1 < error2 ? error1 : error2;
      l1 -= (error1 * error1 + error2 * error2);
    }
    obj->add_coefficient(variables.at(i), l1);
  }

  solver.solve();
  const std::vector<double>& results = solver.solution();

  pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructed_element (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_points (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2( mesh.cloud, *model_points );
  for (std::size_t i = 0; i < results.size(); ++i) {
    if (results[i] != 0){
      std::cout << "\tx" << i + 1 << ": " << results[i] << std::endl;
      std::pair<int, int> edge = face_pairs.at(i);

      std::vector<uint32_t> vertices1 = faces_model[edge.first].vertices;
      std::vector<uint32_t> vertices2 = faces_model[edge.second].vertices;
      for (int j = 0; j < 3; j++){
        reconstructed_element->push_back((*model_points)[vertices1.at(j)]);
        reconstructed_element->push_back((*model_points)[vertices2.at(j)]);
      }
    }
  }
  pcl::io::savePLYFile("/home/philipp/Schreibtisch/reconstructed.ply",*reconstructed_element);

  return 0;
}
