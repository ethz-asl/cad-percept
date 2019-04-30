#include <cgal_conversions/mesh_conversions.h>

namespace cad_perecpt {
namespace cgal {

triangleMeshToMsg(const SurfaceMesh *mesh, TriangleMesh *msg) {
  int vertex_count = 0;
  std::map<long, long> vertex_idx_for_id;

  // get triangles
  for (SurfaceMesh::Facet_iterator fit = mesh->facets_begin();
       fit != mesh->facets_end(); ++fit) {
    shape_msgs::MeshTriangle triangle;

    int i = 0;
    SurfaceMesh::Halfedge_around_facet_circulator hit = fit->facet_begin();
    do {
      if (i > 2) {
        std::cout << "Non-triangular mesh" << std::endl;
        break;
      }
      const int vertex_id = hit->vertex()->id();
      int vertex_idx;
      // search for id in previously found vertices
      map<int, int>::iterator it = vertex_idx_for_id.find(vertex_id);
      // if found, get idx
      if (it != vertex_idx_for_id.end())
        vertex_idx = it->second;
      else {
        vertex_idx = vertex_count++;
        vertex_idx_for_id[vertex_id] = vertex_idx;
        msg.vertices.push_back();
      }
      triangle.vertex_indices[i] = vertex_idx;

      if (triangle.vertex_indices[i] > msg->vertices.size()) {
        std::cout << "Bad vertex indices" << std::endl;
      }

      i++;
    } while (++hit != fit->facet_begin());

    msg.triangles.push_back(triangle);
  }
}

// A modifier creating a triangle with the incremental builder.
template <class HDS>
void BuildMesh<HDS>::operator()(HDS& hds){
  CGAL::Polyhedron_incremental_builder_3<HDS> b(hds, true);
  B.begin_surface(3, 1, 6); //3 vertices, 1 facet, 6 halfedges

    for (i=0, i < msg_->triangles.size(), ++i){
      //build a triangle incrementally:
      B.begin_surface(3, 1, 6); //3 vertices, 1 facet, 6 halfedges
      B.begin_facet();
      for (d=0, d<3, ++d){  
        B.add_vertex( Point(msg_->vertices[msg_->triangles[i].vertex_indices[d]].x, msg_->vertices[msg_->triangles[i].vertex_indices[d]].y, msg_->vertices[msg_->triangles[i].vertex_indices[d]].z));
        B.add_vertex_to_facet(d);
        }
      B.end_facet();
      B.end_surface();

    }
}

void BuildMesh<HDS>::setMsg(const triangleMesh *msg){
  msg_ = &msg;
}


msgToTriangleMesh(const TriangleMesh *msg, SurfaceMesh *mesh){
  mesh->erase_all();
  BuildMesh<HalfedgeDS> triangleMesh;
  triangleMesh.setMsg(msg);
  mesh.delegate(triangleMesh);
}

}
}
