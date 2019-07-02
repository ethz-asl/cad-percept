#ifndef PC_PROCESSING_H_
#define PC_PROCESSING_H_

#include <glog/logging.h>
#include <pcl_ros/point_cloud.h>
#include <boost/circular_buffer.hpp>

#include "pointmatcher/PointMatcher.h"

namespace cad_percept {
namespace room_deviations {

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

}
}

#endif // PC_PROCESSING_H_
