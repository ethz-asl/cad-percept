#ifndef CGAL_DEFINITIONS_CGAL_MESHING_TYPDEFS_H_
#define CGAL_DEFINITIONS_CGAL_MESHING_TYPDEFS_H_

#include <cgal_definitions/cgal_typedefs.h>

// Domain typdefs
#include <CGAL/Labeled_mesh_domain_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_triangulation_3.h>

#include <CGAL/Mesh_3/Robust_intersection_traits_3.h>
#include <CGAL/Mesh_domain_with_polyline_features_3.h>
#include <CGAL/Polyhedral_mesh_domain_3.h>
#include <CGAL/Polyhedral_mesh_domain_with_features_3.h>

namespace cad_percept {
namespace cgal {

// for meshing
typedef CGAL::Labeled_mesh_domain_3<Kernel, int, int> MeshDomain;
typedef CGAL::Polyhedral_mesh_domain_with_features_3<Kernel, Polyhedron, CGAL::Default, int>
    PolyhedronMeshDomain;

typedef CGAL::Kernel_traits<MeshDomain>::Kernel Robust_intersections_traits;
typedef CGAL::details::Mesh_geom_traits_generator<Robust_intersections_traits>::type Robust_K;

typedef CGAL::Compact_mesh_cell_base_3<Robust_K, PolyhedronMeshDomain> Cell_base;
typedef CGAL::Triangulation_cell_base_with_info_3<int, Robust_K, Cell_base> Cell_base_with_info;

#ifdef CONCURRENT_POLYHEDRON_MESH
typedef CGAL::Mesh_triangulation_3<PolyhedronMeshDomain, Robust_intersections_traits,
                                   CGAL::Parallel_tag, CGAL::Default, Cell_base_with_info>::type Tr;
#else
typedef CGAL::Mesh_triangulation_3<PolyhedronMeshDomain, Robust_intersections_traits,
                                   CGAL::Sequential_tag, CGAL::Default, Cell_base_with_info>::type
    Triangulation;
#endif

typedef CGAL::Mesh_complex_3_in_triangulation_3<Triangulation> C3t3;

}  // namespace cgal
}  // namespace cad_percept

#endif  // CGAL_DEFINITIONS_CGAL_MESHING_TYPDEFS_H_
