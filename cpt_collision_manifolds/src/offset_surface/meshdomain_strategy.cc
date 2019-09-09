#include <CGAL/Labeled_mesh_domain_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/make_mesh_3.h>

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <cpt_collision_manifolds/offset_surface/meshdomain_strategy.h>
#include <cpt_utils/perf.h>

namespace cad_percept {
namespace collision_manifolds {
namespace offset_surface {

bool MeshDomainStrategy::execute(const cad_percept::cgal::Polyhedron& surface, const double offset,
                                 cad_percept::cgal::Polyhedron* offset_surface) {
  // adopted from
  // https://github.com/CGAL/cgal/blob/master/Polyhedron/demo/Polyhedron/Plugins/
  // Surface_mesh/Offset_meshing_plugin.cpp

  // Clean passed objects.
  offset_surface->clear();

  // Get bounding box.
  CGAL::Bbox_3 bbox = CGAL::Polygon_mesh_processing::bbox(surface);
  cgal::Point center((bbox.xmax() + bbox.xmin()) / 2, (bbox.ymax() + bbox.ymin()) / 2,
                     (bbox.zmax() + bbox.zmin()) / 2);

  double sqrad = 0.6 * std::sqrt(CGAL::square(bbox.xmax() - bbox.xmin()) +
                                 CGAL::square(bbox.ymax() - bbox.ymin()) +
                                 CGAL::square(bbox.zmax() - bbox.zmin())) +
                 offset;
  sqrad = CGAL::square(sqrad);

  namespace p = CGAL::parameters;

  // Create implicit mesh domain from offset surface.
  OffsetFunction offfunct(surface, offset);

  cgal::MeshDomain domain = cgal::MeshDomain::create_implicit_mesh_domain(
      offfunct, cgal::Sphere(center, sqrad), p::relative_error_bound = 1e-7,
      p::construct_surface_patch_index = [](int i, int j) { return (i * 1000 + j); });

  // Set up mesher with correct topology
  CGAL::Mesh_facet_topology topology = CGAL::FACET_VERTICES_ON_SAME_SURFACE_PATCH;
  if (cfg_->getParam("boundary_mesh", false)) {
    topology = CGAL::Mesh_facet_topology(topology | CGAL::MANIFOLD_WITH_BOUNDARY);
  } else {
    topology = CGAL::Mesh_facet_topology(topology | CGAL::MANIFOLD);
  }
  // read meshing params from config.
  CGAL::Mesh_criteria_3<cgal::C3t3::Triangulation> criteria(
      p::facet_angle = cfg_->getParam("facet_angle", 30.0),
      p::facet_size = cfg_->getParam("facet_size", 0.03),
      p::facet_distance = cfg_->getParam("facet_distance", 0.01), p::facet_topology = topology);

  // Execute mesher and read triangulation.
  Perf::get()->enter("Make_mesh_3");
  cgal::C3t3 c3t3 = CGAL::make_mesh_3<cgal::C3t3>(domain, criteria, p::no_perturb(), p::no_exude());
  const cgal::C3t3::Triangulation& tr = c3t3.triangulation();
  Perf::get()->exit();

  if (tr.number_of_vertices() == 0) {
    return false;
  }

  // Convert triangulatin to Polyhedron.
  Perf::get()->enter("Triangulation");
  CGAL::facets_in_complex_3_to_triangle_mesh(c3t3, *offset_surface);
  Perf::get()->exit();

  // Fix Polyhedron orientation.
  if (CGAL::is_closed(*offset_surface) &&
      !CGAL::Polygon_mesh_processing::is_outward_oriented(*offset_surface)) {
    CGAL::Polygon_mesh_processing::reverse_face_orientations(*offset_surface);
  }

  return true;
}
}  // namespace offset_surface
}  // namespace collision_manifolds
}  // namespace cad_percept
