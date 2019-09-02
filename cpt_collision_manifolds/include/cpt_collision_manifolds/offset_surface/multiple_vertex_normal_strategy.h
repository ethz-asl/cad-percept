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
  typedef std::map<cgal::face_descriptor, cgal::Vector> FaceNormalMap;


  bool execute(const cad_percept::cgal::Polyhedron& surface, double offset,
               cad_percept::cgal::Polyhedron* offset_surface);

 private:
  /*
   * Calculates all face normals
   */
  void getFaceNormals(const cad_percept::cgal::Polyhedron& surface, FaceNormalMap* fnormals);

  /*
   * Evaluates if, for one given vertex, a multiple or a single normal is used and
   * thus, if the vertex is
   * a) just offset as usual (using average normal) (
   * b) Split in two (using multiple normals) and interpolated
   * c) Split in three (using multiple normals) and interpolated
   * d)
   */
  bool createNewVertex(cgal::vertex_descriptor vertex);
};
}  // namespace offset_surface
}  // namespace collision_manifolds
}  // namespace cad_percept

#endif  // CPT_COLLISION_MANIFOLDS_OFFSET_SURFACE_MULTIPLE_VERTEX_NORMAL_STRATEGY_H_
