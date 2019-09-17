#ifndef MESH_CONVERSIONS_H
#define MESH_CONVERSIONS_H

#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_msgs/TriangleMesh.h>
#include <geometry_msgs/Point.h>
#include <pcl_ros/point_cloud.h>

namespace cad_percept {
namespace cgal {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void pointToMsg(const Point &p, geometry_msgs::Point *msg);
geometry_msgs::Point pointToMsg(const Point &p);
void triangleMeshToMsg(Polyhedron &m, cgal_msgs::TriangleMesh *msg);
void msgToTriangleMesh(const cgal_msgs::TriangleMesh &msg, Polyhedron *mesh);
void meshToVerticePointCloud(const Polyhedron &mesh, PointCloud *msg);

template <class HDS>
class BuildMesh : public CGAL::Modifier_base<HDS> {
 public:
  BuildMesh() {}

  void operator()(HDS &hds);
  void setMsg(const cgal_msgs::TriangleMesh &msg);

 private:
  const cgal_msgs::TriangleMesh *msg_;
};
}  // namespace cgal
}  // namespace cad_percept

#endif  // MESH_CONVERSIONS_H
