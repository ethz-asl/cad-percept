#include <CGAL/Surface_mesh_parameterization/Barycentric_mapping_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Discrete_authalic_parameterizer_3.h>
#include <cpt_planning/coordinates/face_coords.h>
#include <cpt_planning/coordinates/uv_mapping.h>

#include <algorithm>
namespace cad_percept {
namespace planning {

void UVMapping::CoordinateMeshBuilder::operator()(cgal::HalfedgeDS &hds) {
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
        LOG(WARNING) << "Tried to parametrized non-triangular mesh, aborting.";
        break;
      }
      B.add_vertex_to_facet(id_translation[hit->vertex()->id()]);

      i++;
    } while (++hit != fit->facet_begin());
    B.end_facet();
  }
  B.end_surface();
}

UVMapping::UVMapping(cgal::MeshModel::Ptr mesh3d, Eigen::Vector3d &zero_point, double zero_angle,
                     CoordinateMapping mapping, BorderParametrization parametrization,
                     BorderShape border)
    : vertex_map_(),
      uv_pmap_(vertex_map_),
      mesh_3d_(mesh3d),
      zero_point_(zero_point),
      zero_angle_(zero_angle) {
  // Following if/elses are a bit ugly, but does the job.
  // not so straighforward due to template class determination

  // Tutte mappings
  if (mapping == CoordinateMapping::TutteBarycentric &&
      parametrization == BorderParametrization::ArcLength && border == BorderShape::Circular) {
    createUVParametrization<TutteCircularArc>();
  } else if (mapping == CoordinateMapping::TutteBarycentric &&
             parametrization == BorderParametrization::Uniform && border == BorderShape::Circular) {
    createUVParametrization<TutteCircularUniform>();
  } else if (mapping == CoordinateMapping::TutteBarycentric &&
             parametrization == BorderParametrization::Uniform && border == BorderShape::Square) {
    createUVParametrization<TutteSquareUniform>();
  } else if (mapping == CoordinateMapping::TutteBarycentric &&
             parametrization == BorderParametrization::ArcLength && border == BorderShape::Square) {
    createUVParametrization<TutteSquareArc>();
  }

  // Authalic mapping
  else if (mapping == CoordinateMapping::DiscreteAuthalic &&
           parametrization == BorderParametrization::ArcLength && border == BorderShape::Circular) {
    createUVParametrization<AuthalicCircularArc>();
  } else if (mapping == CoordinateMapping::DiscreteAuthalic &&
             parametrization == BorderParametrization::Uniform && border == BorderShape::Circular) {
    createUVParametrization<AuthalicCircularUniform>();
  } else if (mapping == CoordinateMapping::DiscreteAuthalic &&
             parametrization == BorderParametrization::Uniform && border == BorderShape::Square) {
    createUVParametrization<AuthalicSquareUniform>();
  } else if (mapping == CoordinateMapping::DiscreteAuthalic &&
             parametrization == BorderParametrization::ArcLength && border == BorderShape::Square) {
    createUVParametrization<AuthalicSquareArc>();
  }

  // Floater mappings
  else if (mapping == CoordinateMapping::FloaterMeanValue &&
           parametrization == BorderParametrization::ArcLength && border == BorderShape::Circular) {
    createUVParametrization<FloaterCircularArc>();
  } else if (mapping == CoordinateMapping::FloaterMeanValue &&
             parametrization == BorderParametrization::Uniform && border == BorderShape::Circular) {
    createUVParametrization<FloaterCircularUniform>();
  } else if (mapping == CoordinateMapping::FloaterMeanValue &&
             parametrization == BorderParametrization::Uniform && border == BorderShape::Square) {
    createUVParametrization<FloaterSquareUniform>();
  } else if (mapping == CoordinateMapping::FloaterMeanValue &&
             parametrization == BorderParametrization::ArcLength && border == BorderShape::Square) {
    createUVParametrization<FloaterSquareArc>();
  }

  // move zero point to determined location
  determineTransformation();

  // Create all the forward/backward hash mappingsf
  createMappings();
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

  uv_transform_.linear() =
      Eigen::AngleAxisd(zero_angle_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  uv_transform_.translation().topRows<2>() = -nominal_zero_uv;
  uv_transform_.translation() = uv_transform_.linear() * uv_transform_.translation();

  LOG(INFO) << "Translation of UV space: " << nominal_zero_uv;
}

void UVMapping::createMappings() {
  CoordinateMeshBuilder builder(mesh_3d_->getMeshRef(), vertex_map_);
  cgal::Polyhedron mesh_2d;
  mesh_2d.delegate(builder);
  mesh_2d_ = std::make_shared<cgal::MeshModel>(mesh_2d, false);
  mesh_2d_->transform(cgal::eigenTransformationToCgalTransformation(uv_transform_.matrix()));

  map_2d_to_3d_.insert(mesh_2d_->getMeshRef().facets_begin(), mesh_2d_->getMeshRef().facets_end(),
                       mesh_3d_->getMeshRef().facets_begin());

  map_3d_to_2d_.insert(mesh_3d_->getMeshRef().facets_begin(), mesh_3d_->getMeshRef().facets_end(),
                       mesh_2d_->getMeshRef().facets_begin());
};

void UVMapping::storeMapping() const {
  // store both meshes
  mesh_2d_->save("mesh2d.off");
  mesh_3d_->save("mesh3d.off");
  LOG(INFO) << "Saved mesh2d.off and mesh3d.off";
}

std::pair<FaceCoords2d, FaceCoords3d> UVMapping::nearestFace(
    cad_percept::cgal::Vector3In vec_in) const {
  FaceCoords3d nearest_3d = nearestFace3D(vec_in);
  return {toUV(nearest_3d), nearest_3d};
}

std::pair<FaceCoords2d, FaceCoords3d> UVMapping::nearestFace(
    cad_percept::cgal::Vector2In vec_in) const {
  FaceCoords2d nearest_2d = nearestFaceUV(vec_in);
  return {nearest_2d, to3D(nearest_2d)};
}

cad_percept::cgal::Vector2Return UVMapping::clipToManifold(
    cad_percept::cgal::Vector2In point_2d) const {
  // get barycentric coords of this point w.r.t nearest face on manifold
  FaceCoords2d face_2d = nearestFaceUV(point_2d);
  Eigen::Vector3d barycentric = face_2d.toBarycentric(point_2d);

  // clip all values to [0.0, 1.0]
  // this results in the closest point inside / the border of the triangle.
  barycentric.x() = std::clamp(barycentric.x(), 0.0, 1.0);
  barycentric.y() = std::clamp(barycentric.y(), 0.0, 1.0);
  barycentric.z() = std::clamp(barycentric.z(), 0.0, 1.0);
  barycentric /= barycentric.sum();

  return face_2d.toCartesian(barycentric);
}

bool UVMapping::onManifold(cad_percept::cgal::Vector2In point_2d) const {
  FaceCoords2d face_2d = nearestFaceUV(point_2d);
  return face_2d.isInside(point_2d);
}

bool UVMapping::onManifold(cad_percept::cgal::Vector3In point_3d) const {
  FaceCoords3d face_3d = nearestFace3D(point_3d);
  return face_3d.isInside(point_3d);
}

Eigen::Vector3d UVMapping::pointUVHto3D(const Eigen::Vector3d &pointuvh) const {
  Eigen::Vector2d point_on_manifold = pointuvh.topRows<2>();

  // get facecoords for this point
  FaceCoords2d face_uv = nearestFaceUV(point_on_manifold);

  // get corresponding 3d triangle
  FaceCoords3d face_3d = to3D(face_uv);

  // translate to 3d point
  Eigen::Vector3d p_xyz = face_uv.translateTo(face_3d, point_on_manifold);

  // add h-dimension contribution (alogn normal)
  p_xyz += pointuvh.z() * face_3d.getNormal();

  return p_xyz;
}

Eigen::Vector3d UVMapping::point3DtoUVH(const Eigen::Vector3d &point3d) const {
  // get point that is on the manifold
  cgal::PointAndPrimitiveId ppid =
      mesh_3d_->getClosestTriangle(point3d.x(), point3d.y(), point3d.z());
  Eigen::Vector3d closest_on_manifold{ppid.first.x(), ppid.first.y(), ppid.first.z()};

  // get Facecoords for this point
  FaceCoords3d face_3d = {ppid.second, mesh_3d_->getMeshRef()};

  // get corresponding 2D triangle
  FaceCoords2d face_uv = toUV(face_3d);

  // translate point on manifold
  Eigen::Vector3d p_uvh;
  Eigen::Vector2d temp = face_3d.translateTo(face_uv, ppid.first);
  p_uvh.topRows<2>() = temp;

  // restore H coordinate as distance to actual point (should be perpendicular?)
  p_uvh.z() = face_3d.getNormal().dot(point3d - closest_on_manifold);

  return p_uvh;
}

FaceCoords2d UVMapping::nearestFaceUV(cad_percept::cgal::Vector2In pt) const {
  cgal::Point_2 pt2d = pt;
  cgal::PointAndPrimitiveId ppid = mesh_2d_->getClosestTriangle({pt2d.x(), pt2d.y(), 0});
  return {ppid.second, mesh_2d_->getMeshRef()};
}

FaceCoords3d UVMapping::nearestFace3D(cad_percept::cgal::Vector3In pt) const {
  cgal::PointAndPrimitiveId ppid = mesh_3d_->getClosestTriangle(pt);
  return {ppid.second, mesh_3d_->getMeshRef()};
}

FaceCoords2d UVMapping::toUV(const cad_percept::planning::FaceCoords3d &coords3d) const {
  return {map_3d_to_2d_[coords3d.getFaceDescriptor()], mesh_2d_->getMeshRef()};
}

FaceCoords3d UVMapping::to3D(const cad_percept::planning::FaceCoords2d &coords2d) const {
  return {map_2d_to_3d_[coords2d.getFaceDescriptor()], mesh_3d_->getMeshRef()};
};

std::string UVMapping::getMappingName() const {
  // Somehow C++ can't convert enum to strings...

  std::stringstream str;
  switch (this->coord_mapping_) {
    case CoordinateMapping::FloaterMeanValue:
      str << "Floater/";
      break;
    case CoordinateMapping::DiscreteAuthalic:
      str << "DiscreteAuthalic/";
      break;
    case CoordinateMapping::TutteBarycentric:
      str << "TuteBarycentric/";
      break;
  }

  switch (this->border_param_) {
    case BorderParametrization::ArcLength:
      str << "ArcLength/";
      break;
    case BorderParametrization::Uniform:
      str << "Uniform/";
      break;
  }

  switch (this->border_shape_) {
    case BorderShape::Circular:
      str << "Circular";
      break;
    case BorderShape::Square:
      str << "Square";
      break;
  }
  return str.str();
}
}  // namespace planning
}  // namespace cad_percept