#include <CGAL/Labeled_mesh_domain_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/make_mesh_3.h>

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <cpt_collision_manifolds/offset_surface/meshdomain_strategy.h>

namespace cad_percept {
namespace collision_manifolds {
namespace offset_surface {

bool MeshDomainStrategy::execute(const cad_percept::cgal::Polyhedron& surface, const double offset,
                                 cad_percept::cgal::Polyhedron* offset_surface) {
  // adopted from
  // https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/
  // Surface_mesh/Offset_meshing_plugin.cpp

  *offset_surface = surface;

  // get bounding box.
  CGAL::Bbox_3 bbox = CGAL::Polygon_mesh_processing::bbox(surface);
  cgal::Point center((bbox.xmax() + bbox.xmin()) / 2, (bbox.ymax() + bbox.ymin()) / 2,
                     (bbox.zmax() + bbox.zmin()) / 2);

  double sqrad = 0.6 * std::sqrt(CGAL::square(bbox.xmax() - bbox.xmin()) +
                                 CGAL::square(bbox.ymax() - bbox.ymin()) +
                                 CGAL::square(bbox.zmax() - bbox.zmin())) +
                 offset;
  sqrad = CGAL::square(sqrad);

  namespace p = CGAL::parameters;

  // Create implicit mesh domain from offset surface
  OffseFunction offfunct(surface, offset);

  cgal::Mesh_domain domain = cgal::Mesh_domain::create_implicit_mesh_domain(
      offfunct, cgal::Sphere(center, sqrad), p::relative_error_bound = 1e-7,
      p::construct_surface_patch_index = [](int i, int j) { return (i * 1000 + j); });

  CGAL::Mesh_facet_topology topology = CGAL::FACET_VERTICES_ON_SAME_SURFACE_PATCH;
  topology = CGAL::Mesh_facet_topology(topology | CGAL::MANIFOLD);
  // topology = CGAL::Mesh_facet_topology(topology | CGAL::MANIFOLD_WITH_BOUNDARY); (if boundaries)

  double angle, sizing, approx;
  CGAL::Mesh_criteria_3<cgal::C3T3::Triangulation> criteria(
      p::facet_angle = angle, p::facet_size = sizing, p::facet_distance = approx,
      p::facet_topology = topology);

  // Todo: Define custom https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/C3t3_type.h

  cgal::C3T3 c3t3 = CGAL::make_mesh_3<cgal::C3T3>(domain, criteria, p::no_perturb(), p::no_exude());

  return false;
}
}  // namespace offset_surface
}  // namespace collision_manifolds
}  // namespace cad_percept
