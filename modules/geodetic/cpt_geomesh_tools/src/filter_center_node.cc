#include <CGAL/IO/Polyhedron_VRML_1_ostream.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/internal/repair_extra.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <cgal_definitions/mesh_model.h>
#include <cpt_geomesh_tools/filter_center_node.h>

#include <fstream>
#include <iostream>

namespace cad_percept::geomesh_tools {

void FilterCenterNode::filter(std::string path, std::pair<double, double> roi_start,
                              std::pair<double, double> roi_end) {
  // read mesh
  cad_percept::cgal::MeshModel::Ptr model;
  cad_percept::cgal::MeshModel::create(path, &model);

  // go through faces, check their edge coordinates, and if outside of bound -> kick out

  auto mesh = model->getMeshNonConst();
  int count = 0;
  int count_out = 0;
  /*std::set<cad_percept::cgal::Polyhedron::Facet_iterator> fs_to_remove;

  for (auto it = mesh->facets_begin(); it != mesh->facets_end(); it++) {
    auto it_pt = it->facet_begin();
    do {
      auto vertex = it_pt->vertex();
      if (vertex->point().x() < roi_start.first || vertex->point().x() > roi_end.first ||
          vertex->point().y() < roi_start.second || vertex->point().y() > roi_end.second) {
        count_out++;
        fs_to_remove.insert(it);

        break;
      }

      it_pt++;
    } while (it_pt != it->facet_begin());
    count++;
  }

  for (auto fi = fs_to_remove.begin(); fi != fs_to_remove.end(); ++fi) {

   mesh->erase_facet((*fi)->facet_begin());
  }*/

  cgal::Point a(roi_start.first, roi_start.second, 0);
  cgal::Point b(roi_end.first, roi_end.second, 0);
  cgal::Vector v_x(1, 0, 0);
  cgal::Vector v_y(0, 1, 0);

  std::cout << count << std::endl;
  std::cout << count_out << std::endl;

  CGAL::Polygon_mesh_processing::remove_self_intersections(*mesh);
  if (CGAL::Polygon_mesh_processing::does_self_intersect(*mesh)) {
    std::cout << "SELF INTERSECT" << std::endl;
  } else {
    std::cout << "NOPE" << std::endl;
  }
  CGAL::Polygon_mesh_processing::clip(*mesh, cgal::Plane(a, -v_x));
  CGAL::Polygon_mesh_processing::clip(*mesh, cgal::Plane(a, -v_y));
  CGAL::Polygon_mesh_processing::clip(*mesh, cgal::Plane(b, v_x));
  CGAL::Polygon_mesh_processing::clip(*mesh, cgal::Plane(b, v_y));
  std::ofstream file;
  file.open("/tmp/output.off", std::ios::binary);
  file << std::fixed;
  file << std::setprecision(15);
  file << *mesh;
  file.flush();
  file.close();
}

}  // namespace cad_percept::geomesh_tools

int main() {
  std::string path = "/external/Datasets/1907_Gornergrat/guillaume_geodata/mesh/zemlya_0.1m.off";
  cad_percept::geomesh_tools::FilterCenterNode node;
  node.filter(path, {869630, 5772350}, {869846, 5772710});
  return 0;
}
