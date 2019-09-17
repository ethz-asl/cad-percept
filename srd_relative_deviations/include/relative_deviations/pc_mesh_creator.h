#ifndef PC_MESH_CREATOR_H
#define PC_MESH_CREATOR_H

#include <CGAL/point_generators_3.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <fstream>
#include <iostream>
#include <random>

namespace cad_percept {
namespace cgal {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void build_sample_polyhedrons(Polyhedron *P, Polyhedron *P_deviated);

}  // namespace cgal
}  // namespace cad_percept

#endif  // PC_MESH_CREATOR_H
