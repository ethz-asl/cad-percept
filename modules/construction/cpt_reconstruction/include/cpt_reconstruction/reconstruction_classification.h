#ifndef CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_CLASSIFICATION_H_
#define CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_CLASSIFICATION_H_

#include "ros/ros.h"

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include "cpt_reconstruction/classified_shapes.h"
#include "cpt_reconstruction/clusters.h"
#include "cpt_reconstruction/shape.h"
#include "std_msgs/String.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <list>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

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
#include <pcl_conversions/pcl_conversions.h>

#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Classification.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Scale_space_reconstruction_3/Advancing_front_mesher.h>
#include <CGAL/Scale_space_reconstruction_3/Jet_smoother.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/jet_smooth_point_set.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/property_map.h>
#include <CGAL/remove_outliers.h>

// Typedefs Scale Space Reconstruction
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_R;
typedef Kernel_R::FT FT;
typedef Kernel_R::Point_3 Point_R;
typedef Kernel_R::Vector_3 Vector_R;
typedef std::pair<Point_R, Vector_R> PointVectorPair_R;

// Typedef Random Forest
typedef CGAL::Simple_cartesian<double> Kernel_M;
typedef Kernel_M::Point_3 Point_M;
typedef CGAL::Surface_mesh<Point_M> Mesh_M;
typedef CGAL::Point_set_3<Point_M> Point_set_M;
typedef CGAL::Classification::Label_handle Label_handle;
typedef CGAL::Classification::Feature_handle Feature_handle;
typedef CGAL::Classification::Label_set Label_set;
typedef CGAL::Classification::Feature_set Feature_set;
typedef CGAL::Classification::Face_descriptor_to_center_of_mass_map<Mesh_M>
    Face_point_map;
typedef CGAL::Classification::Face_descriptor_to_face_descriptor_with_bbox_map<
    Mesh_M>
    Face_with_bbox_map;
typedef CGAL::Classification::Mesh_feature_generator<Kernel_M, Mesh_M,
                                                     Face_point_map>
    Feature_generator;

namespace cad_percept {
namespace cpt_reconstruction {
class Classification {
 public:
  Classification() = delete;
  Classification(ros::NodeHandle nodeHandle1, ros::NodeHandle nodeHandle2);

 private:
  void messageCallback(const ::cpt_reconstruction::clusters &msg);

  /**
   * Computing a reconstructed mesh from points
   */
  void computeReconstructedSurfaceMesh(
      std::vector<PointVectorPair_R> &points, Mesh_M &mesh,
      pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_points);

  /**
   * Classify the faces of the reconstructed mesh
   */
  void classifyMesh(int idx, const std::string config_path, Mesh_M &mesh,
                    std::vector<int> &label_indices);

  // Parameter values
  std::string RF_CONFIG_1_PATH_;
  std::string RF_CONFIG_2_PATH_;
  std::string RF_CONFIG_3_PATH_;
  std::string RF_CONFIG_4_PATH_;
  std::string RF_CONFIG_5_PATH_;
  double CELL_SIZE_;
  int SMOOTHING_ITERATIONS_;
  double MAX_FACET_LENGTH_;
  int NUMBER_OF_SCALES_;
  int N_RING_QUERY_;
  int PREDICTION_METHOD_;

  std::vector<std::string> all_classifier_paths_;

  ros::NodeHandle nodeHandle1_;
  ros::NodeHandle nodeHandle2_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept

#endif  // CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_CLASSIFICATION_H_
