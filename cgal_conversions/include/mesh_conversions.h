#ifndef MESH_CONVERSIONS_H
#define MESH_CONVERSIONS_H

#include <cgal_definitions/cgal_typedefs.h>
#include <CGAL/Polyhedron_icremental_builder_3.h>

typedef SurfaceMesh:HalfedgeDS HalfedgeDS;

template<class HDS>
class BuildMesh : public CGAL::Modifier_base<HDS> {
    public:
    BuildMesh(){}

    void operator()(HDS& hds);
    void setMsg(const TriangleMesh *msg);

    private:
    TriangleMesh *msg_;

};


#endif //MESH_CONVERSIONS_H