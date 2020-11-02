#ifndef PC_PROCESSING_H_
#define PC_PROCESSING_H_

#include <cgal_definitions/cgal_typedefs.h>
#include <glog/logging.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_ros/point_cloud.h>
#include <boost/circular_buffer.hpp>
//#include "pointmatcher/PointMatcher.h"

namespace cad_percept {
namespace cpt_utils {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
/*
 * Kicking out libpointmatcher dependencies for now.
typedef PointMatcher<float> PM;
typedef PM::TransformationParameters TP;
typedef PM::DataPoints DP;


 * Taking sequence of pointclouds from circular buffer which are relatively
 * close and building a map with them. 3D PointClouds need to be pre-aligned
 * in map_frame. Function never tested!

void align_sequence(const boost::circular_buffer<PointCloud> &cb, PointCloud *pointcloud_out);


 * Convert a DP to a PointCloud

PointCloud dpToPointCloud(const DP &dppointcloud);


 * Convert a PointCloud to a DP

DP pointCloudToDP(const PointCloud &pointcloud);
*/
/**
 * Transform a point cloud with an affine transform.
 */
void transformPointCloud(PointCloud *pointcloud, const Eigen::Affine3f &transform);

/**
 * Sample a point cloud from a mesh with a given number of points and noise.
 */
void sample_pc_from_mesh(const cgal::Polyhedron &P, const int no_of_points, const double stddev,
                         PointCloud *pointcloud);

/**
 *  Projecting a PointCloud on a cgal::Plane using a cgal method.
 *  Templated with the cgal Kernel used.
 */
template <typename K>
struct planeProjector {
  typedef typename K::Plane_3 Plane;
  typedef typename K::Point_3 Point;

  void operator()(const PointCloud &cloud_in, const Plane &plane, PointCloud *cloud_out) {
    for (auto point : cloud_in) {
      Point p_proj;
      p_proj = plane.projection(Point(point.x, point.y, point.z));
      cloud_out->push_back(pcl::PointXYZ(p_proj.x(), p_proj.y(), p_proj.z()));
    }
  };
};

/**
 *  The PCL method for projecting a PointCloud to a plane given by ModelCoefficients
 */
void projectToPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                    const pcl::ModelCoefficients::Ptr coefficients,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

/**
 *  Use 2D convex hull to get area
 */
double getArea(const PointCloud &pointcloud);

/**
 * Compute a bounding box of a point cloud.
 */
void computePCBbox(const PointCloud &pointcloud, CGAL::Bbox_3 *bbox);

/**
 *  Estimate of bbox width and height
 */
void bboxDiameters(const PointCloud &pointcloud, double *width, double *height);

/**
 *  Statistical removal of outliers from a pointcloud
 */
void removeOutliers(PointCloud *pointcloud, int knn = 50, float thresh_mult = 1.0);

}  // namespace cpt_utils
}  // namespace cad_percept

#endif  // PC_PROCESSING_H_
