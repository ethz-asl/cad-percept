#include <cgal_definitions/cgal_typedefs.h>

namespace cad_perecpt {
namespace cgal {

triangleMeshToMsg(const SurfaceMesh *m, TriangleMesh *msg) {
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
}
}
