//
// Created by mpantic on 10.06.19.
//

#ifndef CPT_MESHING_DELAUNAY_2D_MESHER_H
#define CPT_MESHING_DELAUNAY_2D_MESHER_H

//CGAL Includes
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

//simplification includes
// Simplification function
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
// Visitor base
#include <CGAL/Surface_mesh_simplification/Edge_collapse_visitor_base.h>

// Stop-condition policy
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
// Non-default cost and placement policies
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Midpoint_and_length.h>

#include <cgal_definitions/cgal_typedefs.h>
#include <cpt_meshing/mesher/delaunay_xd_mesher.h>

namespace cad_percept {
namespace meshing {
namespace Delaunay2DMesher {

typedef CGAL::Exact_predicates_inexact_constructions_kernel
    ReconstructionKernel;

//Local CGAL Typedefs
typedef CGAL::Projection_traits_xy_3<cad_percept::cgal::Kernel>
    ProjectionTraits;

typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int,
                                                    ProjectionTraits>
    TriangulationVertexBase;

typedef CGAL::Triangulation_data_structure_2<TriangulationVertexBase>
    TriangulationDataStructure;

typedef CGAL::Delaunay_triangulation_2<ProjectionTraits,
                                       TriangulationDataStructure>
    TriangulationType;

typedef CGAL::Surface_mesh_simplification::Edge_profile<cad_percept::cgal::Polyhedron>
    EdgeProfile;

typedef DelaunayXDMesher<TriangulationType> Mesher;

}
}
}
#endif //CPT_MESHING_DELAUNAY_2D_MESHER_H
