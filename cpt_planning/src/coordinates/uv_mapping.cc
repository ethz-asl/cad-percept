#include <cpt_planning/coordinates/face_coords.h>
#include <cpt_planning/coordinates/uv_mapping.h>
namespace cad_percept {
namespace planning {

void UVMapping::CoordinateMeshBuilder::operator()(cgal::HalfedgeDS& hds) {
  CGAL::Polyhedron_incremental_builder_3<cgal::HalfedgeDS> B(hds, true);
  B.begin_surface(mesh_.size_of_vertices(), mesh_.size_of_facets());
  // add all vertices

  // translate ids, as this is a simplified mesh (ids are not strictly monotonic anymore, as
  // elements were removed)
  std::map<size_t, size_t> id_translation;
  int id = 0;

  // First add all vertices in 2d
  for (cgal::Polyhedron::Vertex_iterator vtx = mesh_.vertices_begin(); vtx != mesh_.vertices_end();
       ++vtx) {
    // get 2d point
    cgal::Point_2 uv_point = vertexmap_[vtx];
    B.add_vertex(cgal::Point(uv_point.x(), uv_point.y(), 0.0));
    vtx->id() = id;
    id_translation[vtx->id()] = id;
    ++id;
  }

  // add all faces.
  for (cgal::Polyhedron::Face_iterator fit = mesh_.facets_begin(); fit != mesh_.facets_end();
       ++fit) {
    B.begin_facet();
    int i = 0;

    cgal::Polyhedron::Halfedge_around_facet_circulator hit = fit->facet_begin();
    do {
      if (i > 2) {
        std::cout << "Non-triangular mesh" << std::endl;
        break;
      }
      B.add_vertex_to_facet(id_translation[hit->vertex()->id()]);

      i++;
    } while (++hit != fit->facet_begin());
    B.end_facet();
  }
  B.end_surface();
}

void UVMapping::createUVParametrization() {
  namespace SMP = CGAL::Surface_mesh_parameterization;

  // A halfedge on the (possibly virtual) border
  auto bhd = CGAL::Polygon_mesh_processing::longest_border(mesh_3d_->getMeshRef()).first;
  SMP::parameterize(mesh_3d_->getMeshRef(), bhd, uv_pmap_);
}

void UVMapping::determineTransformation() {
  // precondition: UVParametrization has been called.

  // get nominal zero in 3d
  cgal::PointAndPrimitiveId ppid =
      mesh_3d_->getClosestTriangle(cad_percept::cgal::Vector3In(zero_point_));

  // map to 2d
  FaceCoords3d face_3d_(ppid.second, mesh_3d_->getMeshRef());
  TriangleCoords<2> face_2d_ = face_3d_.mapVertices(vertex_map_);

  // adjust 2d for construction
  Eigen::Vector2d nominal_zero_uv = face_3d_.translateTo(face_2d_, ppid.first);
  uv_transform_.translation().topRows<2>() = -nominal_zero_uv;

  std::cout << "Translation " << nominal_zero_uv << std::endl;
}

void UVMapping::createMappings() {
  CoordinateMeshBuilder builder(mesh_3d_->getMeshRef(), vertex_map_);
  cgal::Polyhedron mesh_2d;
  mesh_2d.delegate(builder);
  mesh_2d_ = std::make_shared<cgal::MeshModel>(mesh_2d);
  mesh_2d_->transform(cgal::eigenTransformationToCgalTransformation(uv_transform_.matrix()));

  map_2d_to_3d_.insert(mesh_2d_->getMeshRef().facets_begin(), mesh_2d_->getMeshRef().facets_end(),
                       mesh_3d_->getMeshRef().facets_begin());

  map_3d_to_2d_.insert(mesh_3d_->getMeshRef().facets_begin(), mesh_3d_->getMeshRef().facets_end(),
                       mesh_2d_->getMeshRef().facets_begin());
};

std::pair<FaceCoords2d, FaceCoords3d> UVMapping::nearestFace(cad_percept::cgal::Vector3In vec_in) {
  FaceCoords3d nearest_3d = nearestFace3D(vec_in);
  return {toUV(nearest_3d), nearest_3d};
}

std::pair<FaceCoords2d, FaceCoords3d> UVMapping::nearestFace(cad_percept::cgal::Vector2In vec_in) {
  FaceCoords2d nearest_2d = nearestFaceUV(vec_in);
  return {nearest_2d, to3D(nearest_2d)};
}

FaceCoords2d UVMapping::nearestFaceUV(cad_percept::cgal::Vector2In pt) {
  cgal::Point_2 pt2d = pt;
  cgal::PointAndPrimitiveId ppid = mesh_2d_->getClosestTriangle({pt2d.x(), pt2d.y(), 0});
  return {ppid.second, mesh_2d_->getMeshRef()};
}

FaceCoords3d UVMapping::nearestFace3D(cad_percept::cgal::Vector3In pt) {
  cgal::PointAndPrimitiveId ppid = mesh_3d_->getClosestTriangle(pt);
  return {ppid.second, mesh_3d_->getMeshRef()};
}

FaceCoords2d UVMapping::toUV(const cad_percept::planning::FaceCoords3d& coords3d) {
  return {map_3d_to_2d_[coords3d.getFaceDescriptor()], mesh_2d_->getMeshRef()};
}

FaceCoords3d UVMapping::to3D(const cad_percept::planning::FaceCoords2d& coords2d) {
  return {map_2d_to_3d_[coords2d.getFaceDescriptor()], mesh_3d_->getMeshRef()};
};

}  // namespace planning
}  // namespace cad_percept