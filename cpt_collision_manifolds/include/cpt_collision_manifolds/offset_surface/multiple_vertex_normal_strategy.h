#ifndef CPT_COLLISION_MANIFOLDS_OFFSET_SURFACE_MULTIPLE_VERTEX_NORMAL_STRATEGY_H_
#define CPT_COLLISION_MANIFOLDS_OFFSET_SURFACE_MULTIPLE_VERTEX_NORMAL_STRATEGY_H_
#include <cpt_collision_manifolds/offset_surface/construction_strategy.h>
namespace cad_percept {
namespace collision_manifolds {
namespace offset_surface {
/*
 * Implementation of
 * Kim, S. J., Lee, D. Y., & Yang, M. Y. (2004).,
 * "Offset triangular mesh using the multiple normal vectors of a vertex.",
 * Computer-Aided Design and Applications, 1(1-4), 285-291.
 */
class MultipleVertexNormalStrategy : public ConstructionStrategy {
 public:
  MultipleVertexNormalStrategy() : vertex_statistics_(true) {}
  typedef std::map<cgal::face_descriptor, cgal::Vector> FaceNormalMap;
  typedef std::vector<cgal::Vector> MultiNormal;
  typedef std::map<cgal::vertex_descriptor, MultiNormal> VertexNormalMap;

  bool execute(const cad_percept::cgal::Polyhedron& surface, double offset,
               cad_percept::cgal::Polyhedron* offset_surface);

 private:
  cad_percept::cgal::Polyhedron surface_;
  FaceNormalMap fnormals_;
  VertexNormalMap vnormals_;
  bool vertex_statistics_;
  std::map<uint, uint> vertex_faces_statistics_;

  /*
   * Calculates all face normals
   */
  void getFaceNormals();

  /*
   * Evaluates if, for one given vertex, a multiple or a single normal is used and
   * thus, if the vertex is
   * a) just offset as usual (using average normal) (
   * b) Split in two (using multiple normals) and interpolated
   * c) Split in three (using multiple normals) and interpolated
   * d)
   */
  void calculateMultiNormal(cgal::vertex_descriptor& vertex, MultiNormal* normal);

  /*
   * Todo(mpantic): Remove later
   */
  void moveVertex(std::pair<cgal::vertex_descriptor, MultiNormal> vertex, double offset) const;
};
}  // namespace offset_surface
}  // namespace collision_manifolds
}  // namespace cad_percept

#endif  // CPT_COLLISION_MANIFOLDS_OFFSET_SURFACE_MULTIPLE_VERTEX_NORMAL_STRATEGY_H_
