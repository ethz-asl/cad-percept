# include <cgal_interface/cgal_conversions.h>

namespace cad_percept {
namespace cgal {

void meshToPointCloud(const SurfaceMesh &mesh,
                      PointCloud *pc) {
  pc->width = mesh.size_of_vertices();
  pc->header.frame_id = "mesh";
  pcl::PointXYZ point;
  for (auto vertex = mesh.points_begin();
       vertex != mesh.points_end(); ++vertex) {
    point.x = vertex->x();
    point.y = vertex->y();
    point.z = vertex->z();
    pc->push_back(point);
  }
}

void meshToROSMsg(const SurfaceMesh &mesh,
                      cgal_msgs::ProbabilisticMesh *msg) {

// Transfer points to msg.
  geometry_msgs::Point point;
  for (auto vertex = mesh.points_begin();
       vertex != mesh.points_end(); ++vertex) {
    point.x = vertex->x();
    point.y = vertex->y();
    point.z = vertex->z();
    msg->vertices.push_back(point);
  }
  // Transfer mesh faces to msg.
  shape_msgs::MeshTriangle triangle;
  for (auto facet = mesh.facets_begin();
       facet != mesh.facets_end(); ++facet) { // all mesh surfaces

    if (facet->is_triangle()) {
      triangle.vertex_indices[0] = facet->halfedge()->vertex()->info();
      triangle.vertex_indices[1] = facet->halfedge()->next()->vertex()->info();
      triangle.vertex_indices[2] = facet->halfedge()->opposite()->vertex()
          ->info();

//      for (int i = 0; i < 3; ++i) {
////        Triangle t;
////        t.triangle(facet);
//        triangle.vertex_indices[i] = facet->halfedge()->vertex()->info();
//
////      if (triangle.vertex_indices[i] > used_points) {
////        weird = true;
////      }
//      }

//    if (!weird) {
      msg->triangles.push_back(triangle);
//    }
    }
  }


}
}
}