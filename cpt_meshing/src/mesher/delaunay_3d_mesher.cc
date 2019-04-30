#include <chrono>

#include <cpt_meshing/mesher/delaunay_3d_mesher.h>

namespace cad_percept {
namespace meshing {

bool Delaunay3DMesher::getMesh(cad_percept::cgal::Polyhedron* output,
                               MeshPerformanceCounters* counters = nullptr) {
  if (!inputPointsValid()) {
    return false;
  }

  using clock = std::chrono::steady_clock;
  clock::time_point start = clock::now();

  // set up triangulation datastructure.
  DelaunayTriangulation::Vertex_handle vh;
  DelaunayTriangulation dt;

  std::vector<cgal::Point> vertices;
  vertices.resize(points_->size());

  // perform triangulation
  for (size_t i = 0; i < points_->size(); ++i) {

    // Create Point3 from pcl
    // Todo: Add PCL<->CGAL conversions to cgal_conversions
    vertices[i] =
        cgal::Point(points_->at(i).x, points_->at(i).y, points_->at(i).z);

    vh = dt.insert(vertices[i]);
    vh->info() = i; // Assign index as info.
  }

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

  //  Build a builder and begin construction
  CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
  B.begin_surface(num_vertices, num_facets);

  // add all vertices and check index
  uint test_index = 0;
  for (DelaunayTriangulation::Vertex_iterator
           vit = triangulation_.vertices_begin();
       vit != triangulation_.vertices_end(); ++vit) {
    cgal::Point point = vit->point();
    uint index = vit->info();

    if (index != test_index++) {
      std::cout << "Warning, indexing error during conversion" << std::endl;
    }
    // check assumption
    B.add_vertex(point);
  }

  // Iterate through all triangles and add them
  for (DelaunayTriangulation::Cell_iterator cit = triangulation_.cells_begin();
       cit != triangulation_.cells_end(); ++cit) {

    DelaunayTriangulation::Cell_handle cell = cit;

    // Create new facet and add vertices
    B.begin_facet();
    for (int i = 0; i < 3; ++i) {
      uint vertex_index = cell->vertex(i + 1)->info();
      B.add_vertex_to_facet(vertex_index);
    }
    B.end_facet();
  }

  B.end_surface();
}
}
}