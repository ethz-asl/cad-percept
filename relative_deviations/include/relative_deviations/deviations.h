#ifndef DEVIATIONS_H_
#define DEVIATIONS_H_

#include <unistd.h>
#include <glog/logging.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_conversions/eigen_conversions.h>
#include <cgal_conversions/mesh_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <cpt_utils/pc_processing.h>

// Planar segmentation:
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

// Shape detection:
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <utility> // defines std::pair
#include <list>
#include <CGAL/Timer.h>
#include <CGAL/number_utils.h>

#include <CGAL/point_generators_3.h>

#include <map>
#include <unordered_map>
#include <queue>

#include <boost/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp>
#include <boost/bimap/multiset_of.hpp>

namespace cad_percept {
namespace deviations {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef boost::bimap<boost::bimaps::unordered_set_of<int>, boost::bimaps::multiset_of<int>> association_bimap;
typedef association_bimap::value_type bi_association;

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
  double assocAreaRatioUpperLimit;
  double assocAreaRatioLowerLimit;
  double assocLowerLimitThreshold;
};

struct transformation {
  Eigen::AngleAxisd aa;
  Eigen::Quaterniond quat;
  double distance_score;
};

struct reconstructed_plane {
  Eigen::Vector3d pc_normal;
  std::vector<float> coefficients; // do we still need this?
  PointCloud pointcloud;
};

/**
 *  Saves association of point cloud planes to mesh planes.
 *  ID of merged facet is given in map.
 */ 
struct polyhedron_plane {
  cgal::Plane plane;
  double area;
  Eigen::Vector3d normal;
  double match_score = 0; // match score for pc to mesh plane
  reconstructed_plane rec_plane; // associated point cloud
};

typedef std::map<int,int>::iterator Miterator;
typedef std::multimap<int,int>::iterator Mmiterator;
typedef std::unordered_map<int,polyhedron_plane>::iterator Umiterator;
typedef std::unordered_map<int,int>::iterator Umiterator2;

class Deviations {
  public:
    Deviations();
    ~Deviations();

    parameters params;
    cgal::MeshModel reference_mesh;
    /**
     * Read-in reading pc and execute detection
     */
    void detectChanges(std::vector<reconstructed_plane> *rec_planes_publish, const PointCloud &reading_cloud, std::vector<reconstructed_plane> *remaining_cloud_vector, std::unordered_map<int, transformation> *transformation_map);
    void init(const cgal::Polyhedron &P);
    std::unordered_map<int, polyhedron_plane> plane_map; // plane map saving the ID of coplanar plane associated to plane properties

    /**
     * Bimap saving associations between triangles and planes: Facet ID <-> Plane ID (arbitrary iterated)
    */
    association_bimap bimap;
    
    /**
     * Reset stuff after evaluation of current scan
     */
    void reset();

  private:
    PointCloud ref_pc;

    void planarSegmentationPCL(const PointCloud &cloud_in, std::vector<reconstructed_plane> *rec_planes, PointCloud *remaining_cloud) const;
    void planarSegmentationCGAL(const PointCloud &cloud, std::vector<reconstructed_plane> *rec_planes, PointCloud *remaining_cloud) const;
    template <typename ShapeDetection>
    void runShapeDetection(const PointCloud &cloud, std::vector<reconstructed_plane> *rec_planes, PointCloud *remaining_cloud) const;
    /**
     * Pointcloud plane is associated to corresponding Polyhedron. 
     * Since we know that every point in cloud belongs to the same plane,
     * every cloud point is associated to the same Polyhedron while testing
     * and then a score is evaluated.
     */
    void associatePlane(cgal::MeshModel &mesh_model, const PointCloud &cloud, int *id, double *match_score);
    /**
     * This function finds best association between all p.c. planes and facets based on match_score from associatePlane().
     * Could additionally output non associated facets and point clouds.
     */
    void findBestPlaneAssociation(const std::vector<reconstructed_plane> &cloud_vector, cgal::MeshModel &mesh_model, std::vector<reconstructed_plane> *remaining_plane_cloud_vector);
    void computeFacetNormals(cgal::MeshModel &mesh_model);
    void findPlaneDeviation(std::unordered_map<int, transformation> *transformation_map);

    void initPlaneMap();
    /**
     * Takes list of given reference facet ID's and uses them with colinear facets to perform ICP,
     * P is old (unmerged) Polyhedron.
     * This function is a little bit overcomplicated since we can not directly sample points from
     * polyhedron, but only from triangles.
     */
    void extractReferenceFacets(const int no_of_points, cgal::Polyhedron &P, std::unordered_set<int> &references, PointCloud *icp_pointcloud);

    std::string path_;
};

}
}

#endif // DEVIATIONS_H_
