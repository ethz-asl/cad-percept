#ifndef CPT_PLANNING_UV_MAPPING_H_
#define CPT_PLANNING_UV_MAPPING_H_
#include <CGAL/Surface_mesh_parameterization/parameterize.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/coordinates/face_coords.h>

namespace cad_percept {
namespace planning {

/*
 * Defines a coordinate mapping from a manifold in 3d to a 2d coordinate on that surface.
 * Only locally cartesian
 */
class UVMapping {
  // UV mapping
  typedef CGAL::Unique_hash_map<cgal::face_descriptor, cgal::face_descriptor> FaceHashMap;
  typedef CGAL::Unique_hash_map<cgal::vertex_descriptor, cgal::Point_2> UVVertexMap;
  typedef boost::associative_property_map<UVVertexMap> UVPropertyMap;

  /*
   * Internal class that handles build up of the flattened coordinate representation.
   */
  class CoordinateMeshBuilder : public CGAL::Modifier_base<cgal::HalfedgeDS> {
   public:
    CoordinateMeshBuilder(cgal::Polyhedron &mesh, UVVertexMap &vertexmap)
        : vertexmap_(vertexmap), mesh_(mesh) {}

    void operator()(cgal::HalfedgeDS &hds);

   protected:
    cgal::Polyhedron &mesh_;
    UVVertexMap &vertexmap_;
  };

 public:
  UVMapping(cgal::MeshModel::Ptr mesh3d) : vertex_map_(), uv_pmap_(vertex_map_), mesh_3d_(mesh3d) {
    createUVParametrization();
    createMappings();
  }

  void createUVParametrization();
  void createMappings();

  std::pair<FaceCoords2d, FaceCoords3d> nearestFace(cgal::Vector3In);
  std::pair<FaceCoords2d, FaceCoords3d> nearestFace(cgal::Vector2In);

  FaceCoords2d nearestFaceUV(cgal::Vector2In);

  // Todo: Test if its part of the manifold!
  FaceCoords3d nearestFace3D(cgal::Vector3In);

  /* Mapping functions*/
  FaceCoords2d toUV(const FaceCoords3d &coords3d);

  FaceCoords3d to3D(const FaceCoords2d &coords2d);

 public:
  FaceHashMap map_2d_to_3d_;
  FaceHashMap map_3d_to_2d_;
  UVVertexMap vertex_map_;
  UVPropertyMap uv_pmap_;
  cgal::MeshModel::Ptr mesh_2d_;
  cgal::MeshModel::Ptr mesh_3d_;
};

}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_UV_MAPPING_H_
