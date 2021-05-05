#ifndef CPT_RECONSTRUCTION_MESHGENERATION_H
#define CPT_RECONSTRUCTION_MESHGENERATION_H

#include <geometry_msgs/Vector3.h>
#include "cpt_reconstruction/shape.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <algorithm>
#include <fstream>
#include <vector>

#include <Eigen/Dense>

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
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/poisson_surface_reconstruction.h>

// Source: https://doc.cgal.org/5.0.4/Solver_interface/index.html
#define CGAL_USE_SCIP
#include <CGAL/SCIP_mixed_integer_program_traits.h>
typedef CGAL::SCIP_mixed_integer_program_traits<double> MIP_Solver;
typedef typename MIP_Solver::Variable                        Variable;
typedef typename MIP_Solver::Linear_objective        Linear_objective;
typedef typename MIP_Solver::Linear_constraint        Linear_constraint;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3> Point_with_normal;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

namespace cad_percept {
namespace cpt_reconstruction {
class MeshGeneration {
 public:
  MeshGeneration(ros::NodeHandle nodeHandle_);

 private:
  void messageCallback(const ::cpt_reconstruction::shape &msg);
  bool checkValidPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                       int valid_size, int min_size);
  void fusePlanes();
  void removeSingleDetections();
  void removeConflictingClusters();
  void fit3DPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                  pcl::PolygonMesh &mesh);
  void combineMeshes(const pcl::PolygonMesh &mesh, pcl::PolygonMesh &mesh_all);
  bool preprocessBuildingModel();
  void reconstructElements(pcl::PointCloud<pcl::PointXYZ>::Ptr points_element);

  ros::NodeHandle nodeHandle_;
  ros::Subscriber subscriber_;
  int counter_;
  int received_shapes_;
  std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> kd_trees_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_;
  std::vector<Eigen::Vector3d> ransac_normals_;
  std::vector<int> fusing_count_;

  // Fit plane
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> detected_shapes_points_;
  std::vector<Eigen::Vector4d> detected_shapes_params_;

  // Simplified Model Mesh
  pcl::PolygonMesh mesh_model_;

};
}  // namespace cpt_reconstruction
}  // namespace cad_percept
#endif  // CPT_RECONSTRUCTION_MESHGENERATION_H
