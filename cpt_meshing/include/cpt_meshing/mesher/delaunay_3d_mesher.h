#ifndef CPT_MESHING_DELAUNAY_3D_MESHER_H
#define CPT_MESHING_DELAUNAY_3D_MESHER_H

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Delaunay_triangulation_cell_base_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>

#include <cgal_definitions/cgal_typedefs.h>
#include <cpt_meshing/abstract_simple_mesher.h>

namespace cad_percept {
namespace meshing {

class Delaunay3DMesher : public AbstractSimpleMesher {

  //Local CGAL Typedefs
  typedef CGAL::Triangulation_vertex_base_with_info_3<unsigned int,
                                                      cad_percept::cgal::Kernel>
      TriangulationVertexBase;

  typedef CGAL::Delaunay_triangulation_cell_base_3<cad_percept::cgal::Kernel>
      TriangulationCellBase;

  typedef CGAL::Triangulation_data_structure_3<TriangulationVertexBase,
                                               TriangulationCellBase>
      TriangulationDataStructure;

  typedef CGAL::Delaunay_triangulation_3<cad_percept::cgal::Kernel,
                                         TriangulationDataStructure>
      DelaunayTriangulation;

 public:
  bool getMesh(cad_percept::cgal::SurfaceMesh* output,
               MeshPerformanceCounters* counters);

};
}
}
#endif //CPT_MESHING_DELAUNAY_3D_MESHER_H
