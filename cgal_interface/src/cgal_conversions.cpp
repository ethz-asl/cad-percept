# include <cgal_interface/cgal_conversions.h>

namespace cad_percept {
namespace cgal {

void meshToPointCloud(const SurfaceMesh &mesh,
                      PointCloud *pc) {
//  pc->header
  pc->width = mesh.size_of_vertices();
  pc->header.frame_id = "mesh"
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

//  PointCloud pc;
//  pc.width = P_.size_of_vertices();
//  pcl::PointXYZ point;
//  for (auto vertex = P_.points_begin();
//       vertex != P_.points_end(); ++vertex) {
//    point.x = vertex->x();
//    point.y = vertex->y();
//    point.z = vertex->z();
//    pc.push_back(point);
//  }
//  return pc;


//  std::vector<Kernel::Point_3> points;
//  points.resize(input->size());
//  for (size_t i = 0; i < input->size(); ++i) {
//
//    if (input->at(i).z < 0.2) {
//      continue;
//    }
//
//    points[i] = Kernel::Point_3(input->at(i).x, input->at(i).y, input->at(i).z);
//    vh = dt.insert(points[i]);
//    vh->info() = used_points++;
//
//    geometry_msgs::Point current_point;
//    current_point.x = input->at(i).x;
//    current_point.y = input->at(i).y;
//    current_point.z = input->at(i).z;
//
//    msg->vertices.push_back(current_point);
//    msg->cov_vertices.push_back(geometry_msgs::Point());
//  }
//
//  for (Delaunay::Cell_iterator cit = dt.cells_begin(); cit != dt.cells_end(); ++cit) {
//    shape_msgs::MeshTriangle triangle;
//    bool weird = false;
//
//    for (int i = 0; i < 3; ++i) {
//      triangle.vertex_indices[i] = cell->vertex(i + 1)->info();
//
//      if (triangle.vertex_indices[i] > used_points) {
//        weird = true;
//      }
//    }
//
//    if (!weird) {
//      msg->triangles.push_back(triangle);
//    }

}
}
}