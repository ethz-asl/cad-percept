#ifndef CPT_PLANNING_EVAL_GEODESIC_MESH_PLANNER_H
#define CPT_PLANNING_EVAL_GEODESIC_MESH_PLANNER_H

#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/interface/surface_planner.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel KernelExact;
typedef CGAL::Polyhedron_3<KernelExact, CGAL::Polyhedron_items_with_id_3> PolyhedronExact;
typedef CGAL::Surface_mesh_shortest_path_traits<KernelExact, PolyhedronExact> Traits;
typedef CGAL::Surface_mesh_shortest_path<Traits> Surface_mesh_shortest_path;
typedef CGAL::AABB_face_graph_triangle_primitive<PolyhedronExact>
    PolyhedronExactPrimitive;  // cadify: Primitive
typedef CGAL::AABB_traits<KernelExact, PolyhedronExactPrimitive>
    PolyhedronExactAABBTraits;  // cadify: Traits
typedef CGAL::AABB_tree<PolyhedronExactAABBTraits> PolyhedronEcactAABBTree;

namespace cad_percept {
namespace planning {

// A modifier creating a triangle with the incremental builder.
template <class HDS>
class BuildExactPolyhdron : public CGAL::Modifier_base<HDS> {
 public:
  BuildExactPolyhdron(cgal::MeshModel::Ptr model) : model_(model) {}
  void operator()(HDS& hds) {
    cgal::PolyhedronPtr poly_model = model_->getMeshPtr();
    CGAL::set_halfedgeds_items_id(*poly_model);
    // Postcondition: hds is a valid polyhedral surface.
    CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
    B.begin_surface(poly_model->size_of_vertices(), poly_model->size_of_facets(),
                    poly_model->size_of_halfedges());
    typedef typename HDS::Vertex Vertex;
    typedef typename Vertex::Point Point;
    for (cgal::Polyhedron::Vertex_iterator vti = poly_model->vertices_begin();
         vti != poly_model->vertices_end(); ++vti) {
      Point pt_exact{vti->point().x(), vti->point().y(), vti->point().z()};

      auto v = B.add_vertex(pt_exact);
      v->id() = vti->id();
    }

    for (cgal::Polyhedron::Facet_iterator fti = poly_model->facets_begin();
         fti != poly_model->facets_end(); ++fti) {
      if (!fti->is_triangle()) {
        continue;
      }
      B.begin_facet();

      cgal::Polyhedron::Halfedge_around_facet_circulator hfc = fti->facet_begin();
      // Facets in polyhedral surfaces are at least triangles.
      CGAL_assertion(CGAL::circulator_size(hfc) >= 3);
      do {
        B.add_vertex_to_facet(hfc->vertex()->id());
      } while (++hfc != fti->facet_begin());

      B.end_facet();
    }
    B.end_surface();
  }
  cgal::MeshModel::Ptr model_;
};

/***
 * Implements a SurfacePlanner based on the
 * Discrete Geodesic Algorithm as implemented in CGAL.
 */
class GeodesicMeshPlanner : public SurfacePlanner {
 public:
  GeodesicMeshPlanner(std::string mesh_path);

  const SurfacePlanner::Result plan(const Eigen::Vector3d start, const Eigen::Vector3d goal,
                                    std::vector<Eigen::Vector3d>* states_out);

  inline const std::string getName() const { return "DGEO"; }

  cad_percept::cgal::MeshModel::Ptr model_;
  PolyhedronExact exact_mesh_;
  std::shared_ptr<PolyhedronEcactAABBTree> tree_;
};
}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_EVAL_GEODESIC_MESH_PLANNER_H
