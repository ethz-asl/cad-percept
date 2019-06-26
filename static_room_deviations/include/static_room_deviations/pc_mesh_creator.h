#ifndef PC_MESH_CREATOR_H
#define PC_MESH_CREATOR_H

#include <iostream>
#include <fstream>
#include <random>
#include <cgal_definitions/cgal_typedefs.h>
#include <CGAL/point_generators_3.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

namespace cad_percept {
namespace cgal {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void build_sample_polyhedrons(Polyhedron *P, Polyhedron *P_deviated);
void sample_pc_from_mesh(const Polyhedron &P, 
                         const int no_of_points,
                         const double stddev,
                         PointCloud *pointcloud,
                         std::string file_name);
} 
}

#endif // PC_MESH_CREATOR_H