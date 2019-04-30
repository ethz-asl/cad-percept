#ifndef CGAL_INTERFACE_CGAL_CONVERSIONS_H
#define CGAL_INTERFACE_CGAL_CONVERSIONS_H

#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_msgs/ProbabilisticMesh.h>
#include <pcl_ros/point_cloud.h>

namespace cad_percept {
namespace cgal {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void meshToVerticePointCloud(const Polyhedron &mesh, PointCloud *msg);
}
}

#endif  // CGAL_INTERFACE_CGAL_CONVERSIONS_H
