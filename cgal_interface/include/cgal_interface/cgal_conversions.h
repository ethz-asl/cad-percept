#ifndef CGAL_INTERFACE_CGAL_CONVERSIONS_H
#define CGAL_INTERFACE_CGAL_CONVERSIONS_H


#include <pcl_ros/point_cloud.h>
#include <cgal_msgs/ProbabilisticMesh.h>

#include "cgal_typedefs.h"

namespace cad_percept {
namespace cgal {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void pointcloudToMesh(const SurfaceMesh &mesh,
                      cgal_msgs::ProbabilisticMesh *msg);

void meshToPointCloud(const SurfaceMesh &mesh,
                      PointCloud *msg);
}
}
#endif //CGAL_INTERFACE_CGAL_CONVERSIONS_H