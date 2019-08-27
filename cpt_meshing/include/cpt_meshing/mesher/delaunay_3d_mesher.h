#ifndef CPT_MESHING_DELAUNAY_3D_MESHER_H
#define CPT_MESHING_DELAUNAY_3D_MESHER_H

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Delaunay_triangulation_cell_base_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <cpt_meshing/mesher/delaunay_xd_mesher.h>

namespace cad_percept {
namespace meshing {
namespace Delaunay3DMesher {

typedef CGAL::Exact_predicates_inexact_constructions_kernel
    ReconstructionKernel;

// Local CGAL Typedefs
typedef CGAL::Triangulation_vertex_base_with_info_3<unsigned int,
                                                    ReconstructionKernel>
    TriangulationVertexBase;

typedef CGAL::Delaunay_triangulation_cell_base_3<ReconstructionKernel>
    TriangulationCellBase;

typedef CGAL::Triangulation_data_structure_3<TriangulationVertexBase,
                                             TriangulationCellBase>
    TriangulationDataStructure;

typedef CGAL::Delaunay_triangulation_3<ReconstructionKernel,
                                       TriangulationDataStructure>
    TriangulationType;

typedef DelaunayXDMesher<TriangulationType> Mesher;
}
}
}
#endif  // CPT_MESHING_DELAUNAY_3D_MESHER_H
