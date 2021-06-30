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
#include <glog/logging.h>

namespace cad_percept {
namespace planning {
namespace SMP = CGAL::Surface_mesh_parameterization;
/*
 * Defines a coordinate mapping from a manifold in 3d to a 2d coordinate on that surface.
 * Only locally cartesian.
 */
class UVMapping {
  // UV mapping
  typedef CGAL::Unique_hash_map<cgal::face_descriptor, cgal::face_descriptor> FaceHashMap;
  typedef CGAL::Unique_hash_map<cgal::vertex_descriptor, cgal::Point_2> UVVertexMap;
  typedef boost::associative_property_map<UVVertexMap> UVPropertyMap;

  // CGAL typdefs for the different mappings
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
  // Defines how the coordinates are mapped from 3d to 2d.
  enum class CoordinateMapping { TutteBarycentric, FloaterMeanValue, DiscreteAuthalic };

  // Should the parametrization map to a disc (circular borders) or a square?
  enum class BorderShape { Square, Circular };
  enum class BorderParametrization { Uniform, ArcLength };
  
  UVMapping(cgal::MeshModel::Ptr mesh3d, Eigen::Vector3d &zero_point, double zero_angle = 0.0,
            CoordinateMapping mapping = CoordinateMapping::FloaterMeanValue,
            BorderParametrization parametrization = BorderParametrization::ArcLength,
            BorderShape border = BorderShape::Circular);

  /***
   * Creates the templated parametrization algorithm via CGAL.
   *
   * Define in header file because its templated.
   * @tparam Parametrizer Template type that defines parametrization.
   * @return Time to generate flattened representation.
   */
  template <class Parametrizer>
  std::chrono::microseconds createUVParametrization() {
    namespace SMP = CGAL::Surface_mesh_parameterization;
    auto bhd = CGAL::Polygon_mesh_processing::longest_border(mesh_3d_->getMeshRef()).first;

    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    SMP::parameterize(mesh_3d_->getMeshRef(), Parametrizer(), bhd, uv_pmap_);
    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>((end_time - start_time));
    LOG(INFO) << "Created UV parametrization of type " << getMappingName() << " in "
              << duration.count() << "us.";
    return duration;
  }

  /***
   * Shifts 2D representation to have the set zero point at the origin.
   */
  void determineTransformation();

  /***
   * Creates the hash maps to translate 2d<->3d.
   */
  void createMappings();

  /***
   * Returns neearest face in 2d and 3d for a given 3D coordinate.
   */
  std::pair<FaceCoords2d, FaceCoords3d> nearestFace(cgal::Vector3In) const;

  /***
   * Returns nearest face in 2d and 3d for a given 2D coordinate.
   */
  std::pair<FaceCoords2d, FaceCoords3d> nearestFace(cgal::Vector2In) const;

  /***
   * Returns the closest point within the manifold relative to the given 2d point.
   */
  cad_percept::cgal::Vector2Return clipToManifold(cad_percept::cgal::Vector2In point_2d) const;

  /***
   * Checks if a point is within the manifold based on its 2d coordinate.
   */
  bool onManifold(cad_percept::cgal::Vector2In point_2d) const;

  /***
   * Checks if a point is within the manifold based on its 3d coordinate.
   */
  bool onManifold(cad_percept::cgal::Vector3In point_3d) const;

  Eigen::Vector3d point3DtoUVH(const Eigen::Vector3d &point3d) const;
  Eigen::Vector3d pointUVHto3D(const Eigen::Vector3d &pointUVH) const;

  FaceCoords2d nearestFaceUV(cgal::Vector2In) const;

  /***
   * Returns the nearest 3D triangle based on 3d coordinates.
   */
  FaceCoords3d nearestFace3D(cgal::Vector3In) const;

  /***
   * Maps from 3d to 2d.
   */
  FaceCoords2d toUV(const FaceCoords3d &coords3d) const;

  /***
   * Maps from 2d to 3d.
   */
  FaceCoords3d to3D(const FaceCoords2d &coords2d) const;

  /***
   * Stores both (2d and 3d) meshes as .off files.
   */
  void storeMapping() const;

  /***
   * Returns human readable name of the chosen parametrization.
   */
  std::string getMappingName() const;

 public:
  // mapping ocnfig
  CoordinateMapping coord_mapping_;
  BorderParametrization border_param_;
  BorderShape border_shape_;
  Eigen::Vector3d zero_point_;
  double zero_angle_;
  Eigen::Affine3d uv_transform_{Eigen::Affine3d::Identity()};

  // cgal hash maps to translate 2d<->3d
  FaceHashMap map_2d_to_3d_;
  FaceHashMap map_3d_to_2d_;
  UVVertexMap vertex_map_;
  UVPropertyMap uv_pmap_;

  // 2d and 3d meshes
  cgal::MeshModel::Ptr mesh_2d_;
  cgal::MeshModel::Ptr mesh_3d_;
};

}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_UV_MAPPING_H_
