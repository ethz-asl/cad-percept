#ifndef CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_MESH_GENERATION_H_
#define CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_MESH_GENERATION_H_

#include <geometry_msgs/Vector3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include "cpt_reconstruction/classified_shapes.h"
#include "cpt_reconstruction/clusters.h"
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
  MeshGeneration(ros::NodeHandle nodeHandle);

  enum Semantics {
    WALL = 0,
    CEILING = 1,
    BEAM = 2,
    COLUMN = 3,
    FLOOR = 4,
    CLUTTER = 5
  };

 private:
  void messageCallback(const ::cpt_reconstruction::classified_shapes &msg);

  bool checkShapeConstraints(int sem_class, Eigen::Vector3d &normal);

  void combineMeshes(const pcl::PolygonMesh &mesh, pcl::PolygonMesh &mesh_all);
  void preprocessBuildingModel();
  void computePlanarConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               pcl::PolygonMesh &mesh);
  void computeAllPlanes(std::vector<::pcl::Vertices> &faces_model,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_model,
                        std::vector<Eigen::Vector3d> &plane_normals,
                        std::vector<double> &plane_d, std::vector<double> &area,
                        bool do_normal_correction = true, double eps = 0.1);
  void flagDuplicatedPlanes(std::vector<::pcl::Vertices> &faces_model,
                            std::vector<Eigen::Vector3d> &plane_normals,
                            std::vector<double> &plane_d,
                            std::vector<double> &area,
                            std::vector<int> &duplicated_faces,
                            double min_area = 0.0);

  void processElementCloud(
      int idx, pcl::PointCloud<pcl::PointXYZ>::Ptr element_points,
      pcl::PointCloud<pcl::PointXYZ>::Ptr corners,
      pcl::PointCloud<pcl::PointXYZ>::Ptr corners_top_bottom,
      pcl::PointCloud<pcl::PointXYZ>::Ptr corners_side,
      Eigen::Vector3d &element_normal, double &element_d);

  void selectMainCandidateFaces(
      std::vector<::pcl::Vertices> &faces_model,
      pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_model,
      pcl::PointCloud<pcl::PointXYZ>::Ptr corners,
      Eigen::Vector3d &element_normal, double element_d,
      std::vector<int> &duplicated_faces,
      std::vector<int> &candidate_faces_main,
      std::vector<Eigen::Vector3d> &plane_normals, std::vector<double> &area,
      std::vector<double> &plane_d, double min_area = 0.0,
      bool plane_flipping = true);

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
      std::vector<int> &candidate_faces_ortho_vertical, double min_area = 0.0);

  void computeArtificialVertices(
      std::vector<int> &candidate_faces_main,
      std::vector<int> &candidate_faces_ortho_horizontal,
      std::vector<int> &candidate_faces_ortho_vertical,
      std::vector<Eigen::Vector3d> &plane_normals, std::vector<double> &plane_d,
      Eigen::Vector3d &element_normal, double &element_d,
      pcl::PointCloud<pcl::PointXYZ>::Ptr artificial_vertices,
      std::vector<Eigen::Matrix3d> &shape_directions);

  void planeFromIdx(Eigen::Vector4f &result, int idx,
                    std::vector<Eigen::Vector3d> &plane_normals,
                    std::vector<double> &plane_d);

  void removeDuplicatedPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                              double eps = 10e-6);

  void removeDuplicatedValues(std::vector<double> &vector, double eps = 10e-6);

  bool getReconstructionParameters(
      int idx,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &strong_points_reconstruction,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &weak_points,
      std::vector<Eigen::Vector3d> normal_detected_shapes_vector,
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> element_points_vector,
      std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr>
          element_points_kdtrees_vector,
      std::vector<Eigen::Vector3d> &center_estimates,
      std::vector<Eigen::Matrix3d> &direction_estimates,
      std::vector<std::vector<Eigen::VectorXd>> &parameter_estimates,
      Eigen::Matrix3d &averaged_normals);

  ros::NodeHandle nodeHandle_;
  ros::Subscriber subscriber_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept

#endif  // CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_MESH_GENERATION_H_
