#ifndef PC_PROCESSING_H_
#define PC_PROCESSING_H_

#include <glog/logging.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <boost/circular_buffer.hpp>

#include "pointmatcher/PointMatcher.h"
#include <cgal_definitions/cgal_typedefs.h>

namespace cad_percept {
namespace cpt_utils {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef PointMatcher<float> PM;
typedef PM::TransformationParameters TP;
typedef PM::DataPoints DP;


/**
 * Taking sequence of pointclouds from circular buffer which are relatively
 * close and building a map with them. 3D PointClouds need to be pre-aligned 
 * in map_frame
 */
void align_sequence(const boost::circular_buffer<PointCloud> &cb, PointCloud *pointcloud_out);

PointCloud dpToPointCloud(const DP &dppointcloud);

DP pointCloudToDP(const PointCloud &pointcloud);

void transformPointCloud(PointCloud *pointcloud, const Eigen::Affine3f &transform);

void sample_pc_from_mesh(const cgal::Polyhedron &P, 
                         const int no_of_points,
                         const double stddev,
                         PointCloud *pointcloud);

/**
 *  Projecting a PointCloud on a cgal::Plane using a cgal method.
 */
void projectToPlane(const PointCloud &cloud_in, const cgal::ShapeKernel::Plane_3 &plane, PointCloud *cloud_out);
void projectToPlane(const PointCloud &cloud_in, const cgal::Plane &plane, PointCloud *cloud_out);
/**
 *  The PCL method for projecting a PointCloud to a plane given by ModelCoefficients
 */
void projectToPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, const pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

/**
 *  Perform Principal Component Analysis to get eigenvectors and compute
 *  bounding box from these. Then use bounding box to calculate an estimated
 *  wall center point.
 */
//void pca(const PointCloud &pointcloud, eigenvectors);

/**
 *  Compute the bounding box from pca result
 */
//void getBoundingBox(const PointCloud &pointcloud);

/**
 *  Get an estimated center of plane based on bounding box
 */
//void getCenter(const PointCloud &pointcloud);

/**
 *  Use 2D convex hull to get area
 */
double getArea(const PointCloud &pointcloud);

/**
 *  Estimate of bbox diameter
 */
void bboxDiameters(const PointCloud &pointcloud, double *width, double *height);

void computePCBbox(const PointCloud &pointcloud, CGAL::Bbox_3 *bbox);

/**
 *  Statistical removal of outliers from a pointcloud
 */
void removeOutliers(PointCloud *pointcloud, int knn = 50, float thresh_mult = 1.0);

}
}

#endif // PC_PROCESSING_H_
