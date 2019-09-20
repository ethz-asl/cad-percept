#ifndef DEVIATIONS_H_
#define DEVIATIONS_H_

#include <cgal_conversions/eigen_conversions.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cpt_utils/cpt_utils.h>
#include <cpt_utils/pc_processing.h>
#include <glog/logging.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <unistd.h>

// Planar segmentation:
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>

// Shape detection:
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Timer.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/number_utils.h>
#include <CGAL/pca_estimate_normals.h>
#include <list>
#include <utility>  // defines std::pair

#include <CGAL/Polygon_mesh_processing/bbox.h>

#include <map>
#include <queue>
#include <unordered_map>

#include <pointmatcher/Timer.h>

#include <math.h>
#include <limits>

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
  double matchDistScoreThresh;
  double matchMinDistThresh;
  double matchDistThresh;
  double matchAngleThresh;
  double assocAreaRatioUpperLimit;
  double assocAreaRatioLowerLimit;
  double assocAreaLowerLimitThreshold;
  double minDistWeight;
  double distanceScoreWeight;
  double angleWeight;
  double distWeight;
};

struct transformation {
  int count = 0;         // use count to calculate the average later
  Eigen::AngleAxisd aa;  // remove in case we don't need it, can calculate everything from quat
  Eigen::Quaterniond quat;
  Eigen::Vector3d translation;
  double score;
};

struct reconstructed_plane {
  Eigen::Vector3d pc_normal;
  std::vector<float> coefficients;  // do we still need this?
  PointCloud pointcloud;
};

/**
 *  Saves association of point cloud planes to mesh planes.
 *  ID of merged facet is given in map.
 */
struct polyhedron_plane {
  bool associated = false;  // set true if association was mades
  cgal::Plane plane;
  double area;
  Eigen::Vector3d normal;
  CGAL::Bbox_3 bbox;                                        // bounding box of ref polyhedron_plane
  double match_score = std::numeric_limits<double>::max();  // match score for pc to mesh plane
  reconstructed_plane rec_plane;                            // associated point cloud
  // add normal (for rotation) and translation vector, which we update filter
  // add updated cov. matrix for reconstructed plane, which we can use to get bbox and transl. (this
  // is already an update filter for translations I guess)
};

typedef std::map<int, int>::iterator Miterator;
typedef std::multimap<int, int>::iterator Mmiterator;
typedef std::unordered_map<int, polyhedron_plane>::iterator Umiterator;
typedef std::unordered_map<int, int>::iterator Umiterator2;

class Deviations {
 public:
  Deviations();

  parameters params;
  cgal::MeshModel::Ptr reference_mesh;
  /**
   * Read-in reading pc and execute detection
   */
  void detectChanges(std::vector<reconstructed_plane> *rec_planes_publish,
                     const PointCloud &reading_cloud,
                     std::vector<reconstructed_plane> *remaining_plane_cloud_vector);
  void detectMapChanges(std::vector<reconstructed_plane> *rec_planes, const PointCloud &map_cloud,
                        std::vector<reconstructed_plane> *remaining_plane_cloud_vector,
                        std::unordered_map<int, transformation> *current_transformation_map);
  void init(cgal::Polyhedron &P, const tf::StampedTransform& transform);
  std::unordered_map<int, polyhedron_plane>
      plane_map;  // plane map saving the ID of coplanar plane associated to plane properties
  std::unordered_map<int, transformation>
      transformation_map;  // here we keep all the latest updated transformations

  /**
   * unordered map saving associations between triangles and planes: Facet IDs <-> Plane ID
   * (arbitrary iterated)
   */
  std::unordered_multimap<int, int> planeToFacets;
  std::unordered_map<int, int> facetToPlane;

  /**
   * Reset stuff after evaluation of current scan
   */
  void reset();

 private:
  void planarSegmentationPCL(const PointCloud &cloud_in,
                             std::vector<reconstructed_plane> *rec_planes,
                             PointCloud *remaining_cloud) const;
  void planarSegmentationCGAL(const PointCloud &cloud, std::vector<reconstructed_plane> *rec_planes,
                              PointCloud *remaining_cloud) const;
  template <typename ShapeDetection>
  void runShapeDetection(const PointCloud &cloud, std::vector<reconstructed_plane> *rec_planes,
                         PointCloud *remaining_cloud) const;
  /**
   * Pointcloud plane is associated to corresponding Polyhedron.
   * Since we know that every point in cloud belongs to the same plane,
   * every cloud point is associated to the same Polyhedron while testing
   * and then a score is evaluated.
   */
  bool associatePlane(cgal::MeshModel &mesh_model, const reconstructed_plane &rec_plane, int *id,
                      double *match_score);
  /**
   * This function finds best association between all p.c. planes and facets based on match_score
   * from associatePlane(). Could additionally output non associated facets and point clouds.
   */
  void findBestPlaneAssociation(std::vector<reconstructed_plane> cloud_vector,
                                cgal::MeshModel &mesh_model,
                                std::vector<reconstructed_plane> *remaining_plane_cloud_vector);
  void computeFacetNormals();
  void findPlaneDeviation(std::unordered_map<int, transformation> *current_transformation_map,
                          bool size_check);
  /**
   *  Update filtering of overall transformation_map
   */
  void updateAveragePlaneDeviation(
      const std::unordered_map<int, transformation> &current_transformation_map);

  void initPlaneMap();

  void computeCGALBboxes();

  std::string path_;
};

}  // namespace deviations
}  // namespace cad_percept

#endif  // DEVIATIONS_H_
