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

void triangleMeshToMsg(Polyhedron *m, cgal_msgs::TriangleMesh *msg) {
  // enforce unique IDs per vertice
  CGAL::set_halfedgeds_items_id(*m);

  int vertex_count = 0;
  std::map<int, int> vertex_idx_for_id;

  // get triangles
  for (Polyhedron::Facet_iterator facet = m->facets_begin();
       facet != m->facets_end(); ++facet) {
    if (!facet->is_triangle()) continue;

    shape_msgs::MeshTriangle triangle;

    int i = 0;
    Polyhedron::Halfedge_around_facet_const_circulator hit =
        facet->facet_begin();
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

// A modifier creating a triangle with the incremental builder.
template <class HDS>
void BuildMesh<HDS>::operator()(HDS& hds){
  CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
  B.begin_surface(msg_->vertices.size(), msg_->triangles.size(), 6); //vertices, facets, 6 halfedges
  //add all vertices first
  for (uint i = 0; i < msg_->vertices.size(); ++i){
    B.add_vertex( Point(msg_->vertices[i].x, msg_->vertices[i].y, msg_->vertices[i].z));
  }
  for (uint i = 0; i < msg_->triangles.size(); ++i){
    B.begin_facet();
    for (int j = 0; j<3; ++j){
      B.add_vertex_to_facet(msg_->triangles[i].vertex_indices[j]);
    }
    B.end_facet();
  }
  B.end_surface();
}

template <class HDS>
void BuildMesh<HDS>::setMsg(cgal_msgs::TriangleMesh *msg){
  msg_ = msg;
}

void msgToTriangleMesh(cgal_msgs::TriangleMesh *msg, SurfaceMesh *mesh){
  mesh->erase_all();
  BuildMesh<HalfedgeDS> me;
  me.setMsg(msg);
  mesh->delegate(me);
}

}
}
