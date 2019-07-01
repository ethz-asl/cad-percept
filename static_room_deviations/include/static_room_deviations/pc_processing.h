#ifndef PC_PROCESSING_H_
#define PC_PROCESSING_H_

#include <glog/logging.h>
#include <pcl_ros/point_cloud.h>
#include <boost/circular_buffer.hpp>


namespace cad_percept {
namespace room_deviations {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;



void align_sequence(const boost::circular_buffer<PointCloud> &cb, PointCloud *pointcloud_out);

}
}

#endif // PC_PROCESSING_H_
