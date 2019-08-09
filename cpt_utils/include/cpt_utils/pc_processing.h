#ifndef PC_PROCESSING_H_
#define PC_PROCESSING_H_

#include <glog/logging.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
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
                         PointCloud *pointcloud,
                         std::string file_name);

}
}

#endif // PC_PROCESSING_H_
