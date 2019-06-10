#include <chrono>

#include <cpt_meshing/mesher/delaunay_xd_mesher.h>

namespace cad_percept {
namespace meshing {

template<class DelaunayTriangulation>
bool DelaunayXDMesher<DelaunayTriangulation>::getMesh(cad_percept::cgal::Polyhedron* output,
                                                      MeshPerformanceCounters* counters) {
  if (!inputPointsValid()) {
    return false;
  }

  using clock = std::chrono::steady_clock;
  clock::time_point start = clock::now();

  // set up triangulation datastructure.
  typename DelaunayTriangulation::Vertex_handle vh;
  DelaunayTriangulation dt;

  // perform triangulation
  for (size_t i = 0; i < points_->size(); ++i) {

    // Create Point3 from pcl
    // Todo: Add PCL<->CGAL conversions to cgal_conversions
    typename DelaunayTriangulation::Gt::Point_3
        vertex(points_->at(i).x, points_->at(i).y, points_->at(i).z);

    vh = dt.insert(vertex);
    vh->info() = i; // Assign index as info.
  }

  std::cout << "VERTICES DT: " << dt.number_of_vertices() << std::endl;
  std::cout << "Edges DT: " << dt.number_of_edges() << std::endl;
  std::cout << "Edges DT: " << dt.number_of_cells() << std::endl;
  std::cout << "Edges DT: " << dt.number_of_finite_cells() << std::endl;
  std::cout << "Edges DT: " << dt.number_of_facets() << std::endl;
  std::cout << "Edges DT: " << dt.number_of_finite_facets() << std::endl;


  // convert to SurfaceMesh
  output->erase_all();
  DelaunayXDToPolyhedron<cad_percept::cgal::Polyhedron::HalfedgeDS>
      converter(dt);
  output->delegate(converter);
  CGAL::set_halfedgeds_items_id(*output); // fix ids

  clock::time_point end = clock::now();
  std::chrono::duration<double, std::milli> execution_time = end - start;
  if (counters != nullptr) {
    counters->processing_time = execution_time.count();
  }

  return true;
}

template<class DelaunayTriangulation>
template<class HDS>
void DelaunayXDMesher<DelaunayTriangulation>::DelaunayXDToPolyhedron<HDS>::operator()(
    HDS& hds) {

  size_t num_vertices = triangulation_.number_of_vertices();
  size_t num_facets = triangulation_.number_of_cells();
  std::cout << "NUM_VERTICES: " << num_vertices << std::endl;

  //  Build a builder and begin construction
  CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
  B.begin_surface(num_vertices, num_facets);

  // add all vertices and check index
  for (typename DelaunayTriangulation::Finite_vertices_iterator
           vit = triangulation_.finite_vertices_begin();
       vit != triangulation_.finite_vertices_end(); ++vit) {

    typename DelaunayTriangulation::Gt::Point_3
        r_point = triangulation_.point(vit);

    double x = r_point.x();
    double y = r_point.x();
    double z = r_point.x();

    typename HDS::Vertex_handle vh_new = B.add_vertex(cgal::Point(x, y, z));
    vh_new->id() = vit->info();
    std::cout << vit->info() << std::endl;
  }

  uint cells, facets;
  int errors = 0;
  for (typename DelaunayTriangulation::Finite_edges_iterator
           eit = triangulation_.finite_edges_begin();
       eit != triangulation_.finite_edges_end(); ++eit) {
    std::cout << "EDGE " << eit->first->vertex(0)->info() << " " <<
              eit->first->vertex(1)->info() << std::endl;
  }

  // Iterate through all triangles and add them
  for (typename DelaunayTriangulation::Finite_cells_iterator
           cit = triangulation_.finite_cells_begin();
       cit != triangulation_.finite_cells_end(); ++cit) {

    typename DelaunayTriangulation::Cell_handle cell = cit;


    // Create new facet and add vertices
    B.begin_facet();
    std::cout << "FACET: " << std::endl;
    for (int i = 0; i < 4; ++i) {
      uint vertex_index = cell->vertex(i)->info();

      std::cout << "\tVERTEX " << vertex_index << std::endl;

      std::cout << " " << std::endl;
      if (i > 0) {
        B.add_vertex_to_facet(vertex_index);
      }

    }
    B.end_facet();
  }

  std::cout << "ERRORS = " << errors << std::endl;

  B.end_surface();
}
}
}