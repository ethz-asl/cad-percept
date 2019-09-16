#ifndef CPT_MESHING_DELAUNAY_XD_MESHER_H
#define CPT_MESHING_DELAUNAY_XD_MESHER_H

#include <cgal_definitions/cgal_typedefs.h>
#include <cpt_meshing/abstract_simple_mesher.h>

namespace cad_percept {
namespace meshing {

template <class DelaunayTriangulation>
class DelaunayXDMesher : public AbstractSimpleMesher {
 public:
  // Function has to be implemented in header (non-specialized template)
  bool getMesh(cad_percept::cgal::Polyhedron* output, MeshPerformanceCounters* counters = nullptr) {
    if (!inputPointsValid()) {
      return false;
    }

    using clock = std::chrono::steady_clock;
    clock::time_point start = clock::now();

    // set up triangulation datastructure.
    typename DelaunayTriangulation::Vertex_handle vh;
    DelaunayTriangulation dt;

    // perform triangulation
    for (size_t i = 0; i < points_->size(); ++i) {
      // Create Point3 from pcl
      // Todo: Add PCL<->CGAL conversions to cgal_conversions
      typename DelaunayTriangulation::Point vertex(points_->at(i).x, points_->at(i).y,
                                                   points_->at(i).z);

      vh = dt.insert(vertex);
      vh->info() = i;  // Assign index as info.
    }

    // convert to SurfaceMesh
    output->erase_all();
    DelaunayXDToPolyhedron<cad_percept::cgal::Polyhedron::HalfedgeDS> converter(dt);
    output->delegate(converter);
    CGAL::set_halfedgeds_items_id(*output);  // fix ids

    clock::time_point end = clock::now();
    std::chrono::duration<double, std::milli> execution_time = end - start;
    if (counters != nullptr) {
      counters->processing_time = execution_time.count();
      counters->num_points = points_->size();
      counters->num_vertices = output->size_of_vertices();
    }

    return true;
  }

 protected:
  /* Internal helper class for conversion */
  template <class HDS>
  class DelaunayXDToPolyhedron : public CGAL::Modifier_base<HDS> {
   public:
    DelaunayXDToPolyhedron(const DelaunayTriangulation& triangulation)
        : triangulation_(triangulation) {}

    void operator()(HDS& hds) {
      size_t num_vertices = triangulation_.number_of_vertices();
      size_t num_facets = triangulation_.number_of_faces();

      //  Build a builder and begin construction
      CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
      B.begin_surface(num_vertices, num_facets);

      // add all vertices and check index
      for (typename DelaunayTriangulation::Finite_vertices_iterator vit =
               triangulation_.finite_vertices_begin();
           vit != triangulation_.finite_vertices_end(); ++vit) {
        typename DelaunayTriangulation::Point r_point = triangulation_.point(vit);

        double x = r_point.x();
        double y = r_point.y();
        double z = r_point.z();

        typename HDS::Vertex_handle vh_new = B.add_vertex(cgal::Point(x, y, z));
        vh_new->id() = vit->info();
      }

      uint cells, facets;

      // Iterate through all triangles and add them
      for (typename DelaunayTriangulation::Face_iterator fit = triangulation_.faces_begin();
           fit != triangulation_.faces_end(); ++fit) {
        // Create new facet and add vertices
        B.begin_facet();
        B.add_vertex_to_facet(fit->vertex(0)->info());
        B.add_vertex_to_facet(fit->vertex(1)->info());
        B.add_vertex_to_facet(fit->vertex(2)->info());
        B.end_facet();
      }

      B.end_surface();
    }

   private:
    const DelaunayTriangulation& triangulation_;
  };
};
}
}
#endif  // CPT_MESHING_DELAUNAY_XD_MESHER_H
