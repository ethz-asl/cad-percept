#include <cpt_planning/coordinates/uv_mapping.h>

namespace cad_percept {
namespace planning {

std::pair<FaceCoords2d, FaceCoords3d> UVMapping::nearestFace(cad_percept::cgal::Vector3In vec_in) {
  FaceCoords3d nearest_3d = nearestFace3D(vec_in);
  return {toUV(nearest_3d), nearest_3d};
}

std::pair<FaceCoords2d, FaceCoords3d> UVMapping::nearestFace(cad_percept::cgal::Vector2In vec_in) {
  FaceCoords2d nearest_2d = nearestFaceUV(vec_in);
  return {nearest_2d, to3D(nearest_2d)};
}

FaceCoords2d UVMapping::nearestFaceUV(cad_percept::cgal::Vector2In) {
  return { nullptr, mesh_2d_->getMesh()};
}

FaceCoords3d UVMapping::nearestFace3D(cad_percept::cgal::Vector3In pt) {
  cgal::PointAndPrimitiveId ppid = mesh_3d_->getClosestTriangle(pt);
  cgal::face_descriptor face = ppid.second;
  return {face, mesh_3d_->getMesh()};
}

FaceCoords2d UVMapping::toUV(const cad_percept::planning::FaceCoords3d& coords3d) {
  return {map_3d_to_2d_[coords3d.getFaceDescriptor()], mesh_2d_->getMesh()};
}

FaceCoords3d UVMapping::to3D(const cad_percept::planning::FaceCoords2d& coords2d) {
  return {map_2d_to_3d_[coords2d.getFaceDescriptor()], mesh_3d_->getMesh()};
};

}  // namespace planning
}