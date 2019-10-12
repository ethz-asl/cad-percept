#ifndef DEVIATIONS_H_
#define DEVIATIONS_H_

#include <cgal_conversions/eigen_conversions.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_msgs/GeomDeviation.h>
#include <cpt_utils/conversions.h>
#include <cpt_utils/cpt_utils.h>
#include <cpt_utils/pc_processing.h>
#include <glog/logging.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <unistd.h>
#include <iostream>
#include <map>
#include <queue>
#include <unordered_map>
#include <math.h>
#include <limits>
#include <list>
#include <utility>  // defines std::pair

// PCL:
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// CGAL:
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Timer.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/number_utils.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>

namespace cad_percept {
namespace deviations {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

struct parameters {
  std::string path;
  std::string planarSegmentation;
  std::string planarSegmentationMethod;
  double segmentationDistanceThreshold;
  double segmentationNormalThreshold;
  double segmentationClusterDistance;
  int minNumberOfPlanePoints;
  double segmentationProbability;
  double minPolyhedronArea;
  double matchScoreUpperLimit;
  double matchDistPlaneThresh;
  double matchMinDistThresh;
  double matchDistThresh;
  double matchAngleThresh;
  double assocAreaRatioUpperLimit;
  double assocAreaRatioLowerLimit;
  double assocAreaLowerLimitThreshold;
  double minDistWeight;
  double distPlaneWeight;
  double angleWeight;
  double distWeight;
};

/**
 * Transformation between two facets.
 */
struct transformation {
  int count = 0;         // use count to calculate the average later
  Eigen::AngleAxisd aa;  // remove in case we don't need it, can calculate everything from quat
  Eigen::Quaterniond quat;
  Eigen::Vector3d translation;
  double score;
};

/**
 * Segmented plane.
 */
struct reconstructed_plane {
  Eigen::Vector3d pc_normal;
  std::vector<float> coefficients; 
  PointCloud pointcloud;
};

/**
 *  Model plane. Saves association of segmented planes to mesh planes.
 *  ID of coplanar facets are given in map.
 */
struct polyhedron_plane {
  bool associated = false;  // set true if association was mades
  cgal::Plane plane;
  double area;
  Eigen::Vector3d normal;
  CGAL::Bbox_3 bbox;                                        // bounding box of ref polyhedron_plane
  double match_score = std::numeric_limits<double>::max();  // match score for segm. plane to mesh plane
  reconstructed_plane rec_plane;                            // associated segmented plane
};

class Deviations {
 public:
  Deviations();

  parameters params;
  cgal::MeshModel::Ptr reference_mesh;

  /**
   * Read-in reading pc and execute detection on current scan.
   */
  void detectChanges(std::vector<reconstructed_plane> *rec_planes_publish,
                     const PointCloud &reading_cloud,
                     std::vector<reconstructed_plane> *remaining_plane_cloud_vector);
  /**
   * Read-in map pc and execute detection on complete map. Slow!
   */
  void detectMapChanges(
      std::vector<reconstructed_plane> *rec_planes, const PointCloud &map_cloud,
      std::vector<reconstructed_plane> *remaining_plane_cloud_vector,
      std::unordered_map<std::string, transformation> *current_transformation_map);

  void init(cgal::MeshModel::Ptr &model_ptr, const tf::StampedTransform &transform);

  /**
   * Map with the latest updated transformations.
   */
  std::unordered_map<std::string, transformation>
      transformation_map; 

  /**
   * Map saving associations between facets and planes: Facet IDs <-> Plane ID
   * (arbitrary iterated).
   */
  std::unordered_multimap<std::string, std::string> planeToFacets;
  std::unordered_map<std::string, std::string> facetToPlane;
  
  /**
   * Map saving plane properties for each plane ID.
   */
  std::unordered_map<std::string, polyhedron_plane> plane_map;

  /**
   * Reset stuff after evaluation of current scan.
   */
  void reset();

 private:
  /**
   * Planar segmentation using PCL.
   */
  void planarSegmentationPCL(const PointCloud &cloud_in,
                             std::vector<reconstructed_plane> *rec_planes,
                             PointCloud *remaining_cloud) const;

  /**
   * Planar segmentation using CGAL.
   */
  void planarSegmentationCGAL(const PointCloud &cloud, std::vector<reconstructed_plane> *rec_planes,
                              PointCloud *remaining_cloud) const;
  template <typename ShapeDetection>
  void runShapeDetection(const PointCloud &cloud, std::vector<reconstructed_plane> *rec_planes,
                         PointCloud *remaining_cloud) const;

  /**
   * Segmented plane (point cloud) is being associated to a polyhedron plane.
   * Since we know that every point in cloud belongs to the same plane, every 
   * cloud point is associated to the same polyhedron plane while testing and 
   * then a score is evaluated.
   */
  bool associatePlane(cgal::MeshModel::Ptr &mesh_model, const reconstructed_plane &rec_plane,
                      std::string *id, double *match_score);

  /**
   * Best plane associations between segmented planes and all polyhedron planes 
   * are found using the match score from associatePlane(). Outputs non-associated
   * segmented planes.
   */
  void findBestPlaneAssociation(std::vector<reconstructed_plane> cloud_vector,
                                cgal::MeshModel::Ptr &mesh_model,
                                std::vector<reconstructed_plane> *remaining_plane_cloud_vector);
  /**
   * Compute all facet normals. 
   */                              
  void computeFacetNormals();

  /**
   * Computes every plane deviation from point cloud to model for associated plane pairs 
   * and saves them to a map. A size check on plane area can be made.
   */
  void findPlaneDeviation(
      std::unordered_map<std::string, transformation> *current_transformation_map, bool size_check);

  /**
   * Add the latest transformation result to the average for every model plane. 
   */
  void updateAveragePlaneDeviation(
      const std::unordered_map<std::string, transformation> &current_transformation_map);

  /**
   * Execute some initial computations and associations at the beginning.
   */
  void initPlaneMap();

  /**
   * Compute bounding box of every model plane.
   */
  void computeCGALBboxes();

  std::string path_;
};

}  // namespace deviations
}  // namespace cad_percept

#endif  // DEVIATIONS_H_
