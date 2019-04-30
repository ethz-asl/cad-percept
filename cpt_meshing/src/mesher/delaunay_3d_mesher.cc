#include <chrono>

#include <cpt_meshing/mesher/delaunay_3d_mesher.h>

namespace cad_percept {
namespace meshing {

bool Delaunay3DMesher::getMesh(cad_percept::cgal::SurfaceMesh* output,
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

  for (size_t i = 0; i < points_->size(); ++i) {

    // Create Point3 from pcl
    // Todo: Add PCL<->CGAL conversions to cgal_conversions
    vertices[i] =
        cgal::Point(points_->at(i).x, points_->at(i).y, points_->at(i).z);

    vh = dt.insert(vertices[i]);
    vh->info() = i; // Assign index as info.
  }

  clock::time_point end = clock::now();
  std::chrono::duration<double, std::milli> execution_time = end - start;
  if (counters != nullptr) {
    counters->processing_time = execution_time.count();
  }

}
}
}