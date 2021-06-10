#ifndef CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_MESH_GENERATION_H_
#define CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_MESH_GENERATION_H_

#include <geometry_msgs/Vector3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include "cpt_reconstruction/classified_shapes.h"
#include "cpt_reconstruction/clusters.h"
#include "cpt_reconstruction/element_proposals.h"
#include "cpt_reconstruction/parameters.h"
#include "cpt_reconstruction/shape.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>

#include <pcl/ModelCoefficients.h>
#include <pcl/PCLHeader.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/geometry/quad_mesh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace cad_percept {
namespace cpt_reconstruction {
class MeshGeneration {
 public:
  MeshGeneration() = delete;
  MeshGeneration(ros::NodeHandle nodeHandle1, ros::NodeHandle nodeHandle2);

  enum Semantics {
    WALL = 0,
    CEILING = 1,
    BEAM = 2,
    COLUMN = 3,
    FLOOR = 4,
    CLUTTER = 5
  };

 private:
  ros::NodeHandle nodeHandle1_;
  ros::NodeHandle nodeHandle2_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;

  std::string UPSAMPLED_BUILDING_MODEL_PATH_;
  std::string BUILDING_MODEL_PATH_;
  std::string OUTPUT_DIR_;
  float UPSAMPLED_MODEL_OCTREE_RESOLUTION_;
  double MIN_AREA_;
  double DEFAULT_OFFSET_;
  double DUPLICATE_DOT_PRODUCT_;
  double DUPLICATE_DIFF_D_;

  // Data from msg
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> meshing_clouds_;
  std::vector<int> meshing_classes_;
  std::vector<Eigen::Vector3d> robot_positions_;
  std::vector<Eigen::Vector3d> plane_normals_;
  std::vector<int> ids_;
  std::vector<Eigen::Vector3d> axis_;
  std::vector<double> radius_;

  // Faces and vertices of model and (model + detected shapes)
  pcl::PolygonMesh mesh_model_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_model_only_;
  std::vector<::pcl::Vertices> faces_model_only_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_model_;
  std::vector<::pcl::Vertices> faces_model_;

  // Plane parameters of (model + detected shapes)
  std::vector<double> mesh_plane_d_;
  std::vector<double> mesh_area_;
  std::vector<Eigen::Vector3d> mesh_plane_normals_;

  // List of redundant planes of (model + detected shapes)
  std::vector<int> duplicated_faces_;

  // For detected shapes
  // Vector with arifical vertices (from intersection) and the corresponding
  // kd_tree
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> artificial_vertices_vector_;
  std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr>
      artificial_kdtrees_vector_;
  // Vector with normal (direction considering robot position) and d
  std::vector<Eigen::Vector3d> normal_detected_shapes_vector_;
  std::vector<double> detected_shapes_d_vector_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> element_corners_vector_;
  std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr>
      element_points_kdtrees_vector_;
  // Storing Parameters (3 Vectors) for reconstruction
  std::vector<Eigen::Matrix3d> shape_directions_;

  // Candidates for main direction
  std::vector<int> candidate_faces_main_;
  std::vector<int> candidate_faces_ortho_horizontal_;
  std::vector<int> candidate_faces_ortho_vertical_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr upsampled_model_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr model_octree_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr model_upsampled_kdtree_;

  void messageCallback(const ::cpt_reconstruction::classified_shapes &msg);

  void getMessageData(const ::cpt_reconstruction::classified_shapes &msg,
                      pcl::PolygonMesh &mesh_detected);

  void preprocessFusedMesh(pcl::PolygonMesh &mesh_detected, double min_area);

  void getProposalVerticesPlanes(double min_area);

  void getElementProposalsPlanes(
      std::vector<Eigen::Vector3d> &center_estimates,
      std::vector<Eigen::Matrix3d> &direction_estimates,
      std::vector<std::vector<Eigen::VectorXd>> &parameter_estimates,
      pcl::PointCloud<pcl::PointXYZ>::Ptr strong_points,
      pcl::PointCloud<pcl::PointXYZ>::Ptr weak_points);

  void getElementProposalsCylinders(
      std::vector<Eigen::MatrixXd> &bounded_axis_estimates,
      std::vector<double> &radius_estimates);

  void evaluateProposals(
      pcl::PolygonMesh &resulting_mesh,
      std::vector<Eigen::Vector3d> &center_estimates,
      std::vector<Eigen::Matrix3d> &direction_estimates,
      std::vector<std::vector<Eigen::VectorXd>> &parameter_estimates);

  void getHierarchicalVertices(
      pcl::PointCloud<pcl::PointXYZ>::Ptr strong_points,
      pcl::PointCloud<pcl::PointXYZ>::Ptr weak_points,
      pcl::PointCloud<pcl::PointXYZ>::Ptr backup_points);

  bool checkShapeConstraints(int sem_class, Eigen::Vector3d &normal, pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud,
                             int cur_id);

  void combineMeshes(const pcl::PolygonMesh &mesh, pcl::PolygonMesh &mesh_all);

  void computePlanarConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               pcl::PolygonMesh &mesh, bool include_offsets);

  void computeAllPlanes(bool do_normal_correction = true, double eps = 0.1);

  void flagDuplicatedPlanes(double min_area = 0.0);

  void processElementCloud(
      int idx, pcl::PointCloud<pcl::PointXYZ>::Ptr element_points,
      pcl::PointCloud<pcl::PointXYZ>::Ptr corners,
      pcl::PointCloud<pcl::PointXYZ>::Ptr corners_top_bottom,
      pcl::PointCloud<pcl::PointXYZ>::Ptr corners_side,
      Eigen::Vector3d &element_normal, double &element_d);

  void selectMainCandidateFaces(pcl::PointCloud<pcl::PointXYZ>::Ptr corners,
                                Eigen::Vector3d &element_normal,
                                double min_area = 0.0,
                                bool plane_flipping = true);

  void selectOrthoCandidateFacesWall(
      pcl::PointCloud<pcl::PointXYZ>::Ptr corners_side,
      pcl::PointCloud<pcl::PointXYZ>::Ptr corners_top_bottom,
      Eigen::Vector3d &element_normal, double min_area = 0.0);

  void selectOrthoCandidateFacesFloorCeiling(
      pcl::PointCloud<pcl::PointXYZ>::Ptr corners,
      Eigen::Vector3d &element_normal, double min_area = 0.0);

  void computeArtificialVertices(
      pcl::PointCloud<pcl::PointXYZ>::Ptr artificial_vertices,
      std::vector<Eigen::Matrix3d> &shape_directions);

  void planeFromIdx(Eigen::Vector4f &result, int idx);

  void removeDuplicatedPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                              double eps = 10e-6);

  void removeDuplicatedValues(std::vector<double> &vector, double eps = 10e-6);

  bool getReconstructionParametersPlanes(
      int idx,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &strong_points_reconstruction,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &weak_points,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &backup_points,
      std::vector<Eigen::Vector3d> &center_estimates,
      std::vector<Eigen::Matrix3d> &direction_estimates,
      std::vector<std::vector<Eigen::VectorXd>> &parameter_estimates);
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept

#endif  // CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_MESH_GENERATION_H_
