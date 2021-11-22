#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <cpt_planning_eval/dgeo/geodesic_mesh_planner.h>

namespace cad_percept {
namespace planning {
GeodesicMeshPlanner::GeodesicMeshPlanner(std::string mesh_path) {
  cad_percept::cgal::MeshModel::create(mesh_path, &model_, true);

  // Convert Model to exact reconstruction kernel (needed by geodesic planner)
  BuildExactPolyhdron<PolyhedronExact::HDS> exactBuilder(model_);
  exact_mesh_.delegate(exactBuilder);
  CGAL::set_halfedgeds_items_id(exact_mesh_);
  tree_ = std::make_shared<PolyhedronEcactAABBTree>(CGAL::faces(exact_mesh_).first,
                                                    CGAL::faces(exact_mesh_).second, exact_mesh_);
  tree_->accelerate_distance_queries();
}

const SurfacePlanner::Result GeodesicMeshPlanner::plan(const Eigen::Vector3d start,
                                                       const Eigen::Vector3d goal,
                                                       std::vector<Eigen::Vector3d> *states_out) {
  // build query
  Surface_mesh_shortest_path shortest_paths(exact_mesh_);

  // Locate start and end points in mesh
  Traits::Point_3 start_cgal(start.x(), start.y(), start.z()),
      goal_cgal(goal.x(), goal.y(), goal.z());
  auto start_face_loc = shortest_paths.locate(start_cgal, *tree_);
  auto goal_face_loc = shortest_paths.locate(goal_cgal, *tree_);

  // setup start location and start timer
  shortest_paths.add_source_point(start_face_loc);
  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

  // calculate path and stop timer
  std::vector<Traits::Point_3> points;
  auto resultpath = shortest_paths.shortest_path_points_to_source_points(
      goal_face_loc.first, goal_face_loc.second, std::back_inserter(points));

  std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();

  // Parse results
  for (Traits::Point_3 point : points) {
    states_out->push_back({point.x(), point.y(), point.z()});
  }

  SurfacePlanner::Result result;
  result.success = true;
  result.duration = end_time - start_time;
  return result;
}
}  // namespace planning
}  // namespace cad_percept