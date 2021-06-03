#ifndef CPT_RECONSTRUCTION_CLUSTERING_H
#define CPT_RECONSTRUCTION_CLUSTERING_H

#include <geometry_msgs/Vector3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "cpt_reconstruction/clusters.h"
#include "cpt_reconstruction/shape.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <algorithm>
#include <cmath>
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
/*
#include <CGAL/SCIP_mixed_integer_program_traits.h>
typedef CGAL::SCIP_mixed_integer_program_traits<double> MIP_Solver;
typedef typename MIP_Solver::Variable                        Variable;
typedef typename MIP_Solver::Linear_objective        Linear_objective;
typedef typename MIP_Solver::Linear_constraint        Linear_constraint;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3> Point_with_normal;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
*/

namespace cad_percept {
namespace cpt_reconstruction {
class Clustering {
 public:
  Clustering() = delete;
  Clustering(ros::NodeHandle nodeHandle1, ros::NodeHandle nodeHandle2);

 private:
  std::string SHAPES_PATH_;
  std::string MESHES_PATH_;
  float VOXEL_GRID_FILTER_RESOLUTION_;
  int MIN_SIZE_VALID_PLANE_;
  int VALID_SIZE_THRESHOLD_PLANE_;
  double THRESHOLD_SECOND_EIGENVALUE_;
  int INTERVAL_FUSING_CLUSTERS_;
  int INTERVAL_CLEANING_CLUSTERS_;
  int INTERVAL_FORWARDING_CLUSTERS_;
  double DOT_PRODUCT_NORMALS_;
  double DOT_PRODUCT_AXIS_;
  double DISTANCE_MATCHING_POINT_;
  double COVERAGE_;
  double DISTANCE_CONFLICTING_POINT_;
  double OVERLAP_;
  int DETECTION_COUNT_;

  void messageCallback(const ::cpt_reconstruction::shape &msg);
  bool checkValidPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                       int valid_size, int min_size);
  void fusePlanes();
  void fuseCylinders();
  void removeSingleDetectionsPlanes();
  void removeSingleDetectionsCylinders();
  void removeConflictingClustersPlanes();
  void fit3DPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                  pcl::PolygonMesh &mesh);
  void combineMeshes(const pcl::PolygonMesh &mesh, pcl::PolygonMesh &mesh_all);

  ros::NodeHandle nodeHandle1_;
  ros::NodeHandle nodeHandle2_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  int counter_;

  // Planes
  int received_shapes_plane_;
  std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> kd_trees_plane_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_plane_;
  std::vector<Eigen::Vector3d> ransac_normals_;
  std::vector<int> fusing_count_plane_;
  std::vector<Eigen::Vector3d> robot_positions_;

  // Cylinders
  int received_shapes_cyl_;
  std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> kd_trees_cyl_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_cyl_;
  std::vector<Eigen::Vector3d> axis_;
  std::vector<double> radius_;
  std::vector<int> fusing_count_cyl_;

  // Fit plane
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> detected_shapes_points_;
  std::vector<Eigen::Vector4d> detected_shapes_params_;

  // Simplified Model Mesh
  // pcl::PolygonMesh mesh_model_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept
#endif  // CPT_RECONSTRUCTION_CLUSTERING_H
