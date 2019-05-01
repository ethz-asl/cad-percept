#ifndef MESH_CONVERSIONS_H
#define MESH_CONVERSIONS_H

#include <cgal_definitions/cgal_typedefs.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <geometry_msgs/Point.h>
#include <cgal_msgs/TriangleMesh.h>

namespace cad_percept {
namespace cgal {

typedef Polyhedron::HalfedgeDS HalfedgeDS;

void vertexToPointMsg(const Point *vertex, geometry_msgs::Point *msg);
geometry_msgs::Point vertexToPointMsg(const Point *vertex);
void triangleMeshToMsg(Polyhedron *m, cgal_msgs::TriangleMesh *msg);
void msgToTriangleMesh(cgal_msgs::TriangleMesh *msg, Polyhedron *mesh);

template<class HDS>
class BuildMesh : public CGAL::Modifier_base<HDS> {
    public:
    BuildMesh(){}

    void operator()(HDS& hds);
    void setMsg(cgal_msgs::TriangleMesh *msg);

    private:
    cgal_msgs::TriangleMesh *msg_;

};

void triangleMeshToMsg(Polyhedron *m, cgal_msgs::TriangleMesh *msg);
}
}

#endif //MESH_CONVERSIONS_H