#ifndef CPT_PLANNING_UV_MAPPING_H_
#define CPT_PLANNING_UV_MAPPING_H_
#include <CGAL/Surface_mesh_parameterization/ARAP_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Barycentric_mapping_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Circular_border_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Discrete_authalic_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Square_border_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/parameterize.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cpt_planning/coordinates/face_coords.h>

namespace cad_percept {
namespace planning {
namespace SMP = CGAL::Surface_mesh_parameterization;
/*
 * Defines a coordinate mapping from a manifold in 3d to a 2d coordinate on that surface.
 * Only locally cartesian
 */
class UVMapping {
  // UV mapping
  typedef CGAL::Unique_hash_map<cgal::face_descriptor, cgal::face_descriptor> FaceHashMap;
  typedef CGAL::Unique_hash_map<cgal::vertex_descriptor, cgal::Point_2> UVVertexMap;
  typedef boost::associative_property_map<UVVertexMap> UVPropertyMap;

  typedef SMP::Circular_border_arc_length_parameterizer_3<cgal::Polyhedron> CircularArcBorder;
  typedef SMP::Circular_border_uniform_parameterizer_3<cgal::Polyhedron> CircularUniformBorder;
  typedef SMP::Square_border_uniform_parameterizer_3<cgal::Polyhedron> SquareUniformBorder;
  typedef SMP::Square_border_arc_length_parameterizer_3<cgal::Polyhedron> SquareArcBorder;

  typedef SMP::Barycentric_mapping_parameterizer_3<cgal::Polyhedron, CircularArcBorder>
      TutteCircularArc;
  typedef SMP::Barycentric_mapping_parameterizer_3<cgal::Polyhedron, CircularUniformBorder>
      TutteCircularUniform;
  typedef SMP::Barycentric_mapping_parameterizer_3<cgal::Polyhedron, SquareUniformBorder>
      TutteSquareUniform;
  typedef SMP::Barycentric_mapping_parameterizer_3<cgal::Polyhedron, SquareArcBorder>
      TutteSquareArc;

  typedef SMP::Discrete_authalic_parameterizer_3<cgal::Polyhedron, CircularArcBorder>
      AuthalicCircularArc;
  typedef SMP::Discrete_authalic_parameterizer_3<cgal::Polyhedron, CircularUniformBorder>
      AuthalicCircularUniform;
  typedef SMP::Discrete_authalic_parameterizer_3<cgal::Polyhedron, SquareUniformBorder>
      AuthalicSquareUniform;
  typedef SMP::Discrete_authalic_parameterizer_3<cgal::Polyhedron, SquareArcBorder>
      AuthalicSquareArc;

  typedef SMP::Mean_value_coordinates_parameterizer_3<cgal::Polyhedron, CircularArcBorder>
      FloaterCircularArc;
  typedef SMP::Mean_value_coordinates_parameterizer_3<cgal::Polyhedron, CircularUniformBorder>
      FloaterCircularUniform;
  typedef SMP::Mean_value_coordinates_parameterizer_3<cgal::Polyhedron, SquareUniformBorder>
      FloaterSquareUniform;
  typedef SMP::Mean_value_coordinates_parameterizer_3<cgal::Polyhedron, SquareArcBorder>
      FloaterSquareArc;

  /*
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
  UVMapping(cgal::MeshModel::Ptr mesh3d, Eigen::Vector3d zero_point, double zero_angle = 0.0,
            int method = 8)
      : vertex_map_(),
        uv_pmap_(vertex_map_),
        mesh_3d_(mesh3d),
        zero_point_(zero_point),
        zero_angle_(zero_angle) {
    // ugly but well
    switch (method) {
      case 0:
        createUVParametrization<TutteCircularArc>();
        break;

      case 1:
        createUVParametrization<TutteCircularUniform>();
        break;

      case 2:
        createUVParametrization<TutteSquareUniform>();
        break;

      case 3:
        createUVParametrization<TutteSquareArc>();
        break;

      case 4:
        createUVParametrization<AuthalicCircularArc>();
        break;

      case 5:
        createUVParametrization<AuthalicCircularUniform>();
        break;

      case 6:
        createUVParametrization<AuthalicSquareUniform>();
        break;

      case 7:
        createUVParametrization<AuthalicSquareArc>();
        break;

      case 8:
        createUVParametrization<FloaterCircularArc>();
        break;

      case 9:
        createUVParametrization<FloaterCircularUniform>();
        break;

      case 10:
        createUVParametrization<FloaterSquareUniform>();
        break;

      case 11:
        createUVParametrization<FloaterSquareArc>();
        break;
    }

    determineTransformation();
    createMappings();
  }

  template <class Parametrizer>
  void createUVParametrization() {
    namespace SMP = CGAL::Surface_mesh_parameterization;
    auto bhd = CGAL::Polygon_mesh_processing::longest_border(mesh_3d_->getMeshRef()).first;

    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    SMP::parameterize(mesh_3d_->getMeshRef(), Parametrizer(), bhd, uv_pmap_);
    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::cout << "UVPARAMETRIZATION TIME = "
              << std::chrono::duration_cast<std::chrono::microseconds>(
                     (end_time - start_time))
                     .count()
              << std::endl;
  }

  void determineTransformation();
  void createMappings();

  std::pair<FaceCoords2d, FaceCoords3d> nearestFace(cgal::Vector3In) const;
  std::pair<FaceCoords2d, FaceCoords3d> nearestFace(cgal::Vector2In) const;

  bool onManifold(cad_percept::cgal::Vector2In point_2d) const;
  bool onManifold(cad_percept::cgal::Vector3In point_3d) const;

  Eigen::Vector3d point3DtoUVH(const Eigen::Vector3d &point3d) const;
  Eigen::Vector3d pointUVHto3D(const Eigen::Vector3d &pointUVH) const;

  FaceCoords2d nearestFaceUV(cgal::Vector2In) const;

  // Todo: Test if its part of the manifold!
  FaceCoords3d nearestFace3D(cgal::Vector3In) const;

  /* Mapping functions*/
  FaceCoords2d toUV(const FaceCoords3d &coords3d) const;

  FaceCoords3d to3D(const FaceCoords2d &coords2d) const;

 public:
  FaceHashMap map_2d_to_3d_;
  FaceHashMap map_3d_to_2d_;
  UVVertexMap vertex_map_;
  UVPropertyMap uv_pmap_;
  cgal::MeshModel::Ptr mesh_2d_;
  cgal::MeshModel::Ptr mesh_3d_;

  Eigen::Vector3d zero_point_;
  double zero_angle_;
  Eigen::Affine3d uv_transform_{Eigen::Affine3d::Identity()};
};

}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_UV_MAPPING_H_
