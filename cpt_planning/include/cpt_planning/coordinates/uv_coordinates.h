#ifndef CPT_PLANNING_UV_COORDINATES_H_
#define CPT_PLANNING_UV_COORDINATES_H_
#include <cgal_definitions/cgal_typedefs.h>

namespace cad_percept {
namespace planning {

/*
 * Defines a coordinate mapping from a manifold in 3d to a 2d coordinate on that surface.
 * Only locally cartesian
 */
class UVCoordinates {
  // UV mapping
  typedef CGAL::Unique_hash_map<cgal::vertex_descriptor, cgal::Point_2> UVHashMap;
  typedef boost::associative_property_map<UVHashMap> UVPropertyMap;

  // Reverse UV Mapping (flat mesh to 3d mesh)
  typedef CGAL::Unique_hash_map<cgal::vertex_descriptor, cgal::vertex_descriptor> UVReverseHashMap;

  /*
   * Internal class that handles build up of the flattened coordinate representation.
   */
  class CoordinateMeshBuilder : public CGAL::Modifier_base<cgal::HalfedgeDS> {
   public:
    CoordinateMeshBuilder(cgal::PolyhedronPtr mesh, UVHashMap &mapping,
                          UVReverseHashMap *reverse_mapping);

    cgal::PolyhedronPtr mesh_;
    UVHashMap &mapping_;
    UVReverseHashMap *reverse_mapping_;

    void operator()(HDS &hds);
  };

 public:
};

}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_UV_COORDINATES_H_
