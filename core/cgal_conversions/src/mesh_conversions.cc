#include <cgal_conversions/mesh_conversions.h>

namespace cad_percept {
namespace cgal {

void pointToMsg(const Point &p, geometry_msgs::Point *msg) {
  msg->x = p.x();
  msg->y = p.y();
  msg->z = p.z();
}

geometry_msgs::Point pointToMsg(const Point &p) {
  geometry_msgs::Point msg;
  pointToMsg(p, &msg);
  return msg;
}

void triangleMeshToMsg(Polyhedron &m, cgal_msgs::TriangleMesh *msg) {
  // enforce unique IDs per vertice
  CGAL::set_halfedgeds_items_id(m);

  int vertex_count = 0;
  std::map<int, int> vertex_idx_for_id;

  // get triangles
  for (Polyhedron::Facet_iterator facet = m.facets_begin(); facet != m.facets_end(); ++facet) {
    if (!facet->is_triangle()) continue;

    shape_msgs::MeshTriangle triangle;

    int i = 0;
    Polyhedron::Halfedge_around_facet_const_circulator hit = facet->facet_begin();
    do {
      if (i > 2) {
        std::cout << "Non-triangular mesh" << std::endl;
        break;
      }
      const int vertex_id = hit->vertex()->id();
      int vertex_idx;
      // search for id in previously found vertices
      std::map<int, int>::iterator it = vertex_idx_for_id.find(vertex_id);
      // if found, get idx
      if (it != vertex_idx_for_id.end())
        vertex_idx = it->second;
      else {
        vertex_idx = vertex_count++;
        vertex_idx_for_id[vertex_id] = vertex_idx;
        msg->vertices.push_back(pointToMsg(hit->vertex()->point()));
      }
      triangle.vertex_indices[i] = vertex_idx;

      if (triangle.vertex_indices[i] > msg->vertices.size()) {
        std::cout << "Bad vertex indices" << std::endl;
      }

      i++;
    } while (++hit != facet->facet_begin());

    msg->triangles.push_back(triangle);
  }
}

void meshModelToMsg(const MeshModel::Ptr &model, cgal_msgs::TriangleMesh *msg) {
  Polyhedron p = model->getMesh();
  triangleMeshToMsg(p, msg);
  // Iterate in the same order through triangles again and get their ids
  for (Polyhedron::Facet_iterator facet = p.facets_begin(); facet != p.facets_end(); ++facet) {
    if (!facet->is_triangle()) continue;
    msg->triangle_ids.push_back(model->getIdFromFacetHandle(facet));
  }
}

// A modifier creating a triangle with the incremental builder.
template <class HDS>
void BuildMesh<HDS>::operator()(HDS &hds) {
  CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
  B.begin_surface(msg_->vertices.size(), msg_->triangles.size());  // vertices, facets, halfedges
  // add all vertices first
  for (auto const &vertice : msg_->vertices) {
    B.add_vertex(Point(vertice.x, vertice.y, vertice.z));
  }
  for (auto const &triangle : msg_->triangles) {
    B.begin_facet();
    for (int j = 0; j < 3; ++j) {
      B.add_vertex_to_facet(triangle.vertex_indices[j]);
    }
    B.end_facet();
  }
  B.end_surface();
}

template <class HDS>
void BuildMesh<HDS>::setMsg(const cgal_msgs::TriangleMesh &msg) {
  msg_ = &msg;
}

void msgToTriangleMesh(const cgal_msgs::TriangleMesh &msg, Polyhedron *mesh) {
  mesh->erase_all();
  BuildMesh<HalfedgeDS> mesh_generator;
  mesh_generator.setMsg(msg);
  mesh->delegate(mesh_generator);
}

void msgToMeshModel(const cgal_msgs::TriangleMesh &msg, MeshModel::Ptr *model_ptr) {
  Polyhedron P;
  msgToTriangleMesh(msg, &P);
  cgal::MeshModel::create(P, model_ptr);
  if (msg.triangle_ids.size() > 0) {
    (*model_ptr)->setTriangleIds(msg.triangle_ids);
  }
}

void meshToVerticePointCloud(const Polyhedron &mesh, PointCloud *pc) {
  pc->width = mesh.size_of_vertices();  // number of vertices
  pc->header.frame_id = "mesh";
  pcl::PointXYZ point;
  for (auto vertex_point = mesh.points_begin(); vertex_point != mesh.points_end(); ++vertex_point) {
    point.x = vertex_point->x();
    point.y = vertex_point->y();
    point.z = vertex_point->z();
    pc->push_back(point);
  }
}
}  // namespace cgal
}  // namespace cad_percept
