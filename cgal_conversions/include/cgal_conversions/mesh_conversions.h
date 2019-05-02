#include <CGAL/Polyhedron_items_with_id_3.h>
#include <geometry_msgs/Point.h>

#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_msgs/TriangleMesh.h>

namespace cad_percept {
namespace cgal {
void vertexToPointMsg(const Point *vertex, geometry_msgs::Point *msg);
geometry_msgs::Point vertexToPointMsg(const Point *vertex);
void triangleMeshToMsg(Polyhedron *m, cgal_msgs::TriangleMesh *msg);
}
}
