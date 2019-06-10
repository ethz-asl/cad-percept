#include <chrono>

#include <cpt_meshing/mesher/delaunay_3d_mesher.h>

namespace cad_percept {
namespace meshing {

bool Delaunay3DMesher::getMesh(cad_percept::cgal::Polyhedron* output,
                               MeshPerformanceCounters* counters) {
  if (!inputPointsValid()) {
    return false;
  }

  using clock = std::chrono::steady_clock;
  clock::time_point start = clock::now();

  // set up triangulation datastructure.
  DelaunayTriangulation::Vertex_handle vh;
  DelaunayTriangulation dt;


  vertices_.resize(points_->size());

  // perform triangulation
  for (size_t i = 0; i < points_->size(); ++i) {

    // Create Point3 from pcl
    // Todo: Add PCL<->CGAL conversions to cgal_conversions
    vertices_[i] =
        cgal::Point(points_->at(i).x, points_->at(i).y, points_->at(i).z);

    vh = dt.insert(vertices_[i]);
    vh->info() = i; // Assign index as info.
  }

  std::cout <<"VERTICES_: "<< vertices_.size() << std::endl;


  // convert to SurfaceMesh
  output->erase_all();
  Delaunay3DToPolyhedron<cad_percept::cgal::Polyhedron::HalfedgeDS>
      converter(dt);
  output->delegate(converter);

  clock::time_point end = clock::now();
  std::chrono::duration<double, std::milli> execution_time = end - start;
  if (counters != nullptr) {
    counters->processing_time = execution_time.count();
  }

  return true;
}

template<class HDS>
void Delaunay3DMesher::Delaunay3DToPolyhedron<HDS>::operator()(HDS& hds) {

  size_t num_vertices = triangulation_.number_of_vertices();
  size_t num_facets = triangulation_.number_of_cells();
  std::cout <<"NUM_VERTICES: "<< num_vertices << std::endl;

  //  Build a builder and begin construction
  CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
  B.begin_surface(num_vertices, num_facets);

  // add all vertices and check index
  for (DelaunayTriangulation::Finite_vertices_iterator
           vit = triangulation_.finite_vertices_begin();
       vit != triangulation_.finite_vertices_end(); ++vit) {

    typename HDS::Vertex_handle vh_new = B.add_vertex(triangulation_.point(vit));
    vh_new->id() = vit->info();
    std::cout << vit->info() << std::endl;
  }

  uint cells, facets;
  int errors = 0;

  // Iterate through all triangles and add them
  for (DelaunayTriangulation::Finite_cells_iterator cit = triangulation_.finite_cells_begin();
       cit != triangulation_.finite_cells_end(); ++cit) {

    DelaunayTriangulation::Cell_handle cell = cit;


    // Create new facet and add vertices
    B.begin_facet();
    for (int i = 0; i < 3; ++i) {
      uint vertex_index = cell->vertex(i + 1)->info();
    /*  if(vertex_index >= num_vertices){

        std::cout << "VERTEX ERROR " << vertex_index << " " << num_vertices << std::endl;
      }*/
      if(vertex_index == 0){
        std::cout << "VERTEX ERROR " << vertex_index << " " << num_vertices << std::endl;
        errors++;
      }
      B.add_vertex_to_facet(vertex_index);


    }
    B.end_facet();
  }

  std::cout << "ERRORS = " << errors << std::endl;
  B.remove_unconnected_vertices();

  B.end_surface();
}
}
}