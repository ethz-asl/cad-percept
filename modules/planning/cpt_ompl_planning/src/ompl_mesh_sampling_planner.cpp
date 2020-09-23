#include <CGAL/point_generators_3.h>
#include <cpt_ompl_planning/ompl_mesh_sampling_planner.h>
#include <cpt_ompl_planning/rmp_mesh_planner.h>
#include <cpt_ros/mesh_model_publisher.h>

#include <chrono>

ros::Publisher vis_pub_;

void visualizePath(std::vector<Eigen::Vector3d> path, Eigen::Vector3d color, int id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "path";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.0;
  marker.scale.z = 0.0;
  marker.color.a = 1.0;  // Don't forget to set the alpha!
  marker.color.r = color.x();
  marker.color.g = color.y();
  marker.color.b = color.z();

  for (auto state : path) {
    geometry_msgs::Point pt;
    pt.x = state.x();
    pt.y = state.y();
    pt.z = state.z();
    marker.points.push_back(pt);
  }

  vis_pub_.publish(marker);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ompl_test_node");
  ros::NodeHandle node_handle;
  cad_percept::cgal::MeshModel::Ptr model;
  cad_percept::cgal::MeshModel::create(
      "/home/mpantic/ws/rmp/src/manifold_simulations/models/hilo_roof/meshes/"
      "hilo_reconstructed.off",
      &model, true);
  cad_percept::MeshModelPublisher pub_mesh_3d_(node_handle, "mesh");
  pub_mesh_3d_.publish(model);

  auto rng_mesh_ =
      std::make_shared<CGAL::Random_points_in_triangle_mesh_3<cad_percept::cgal::Polyhedron>>(
          model->getMeshRef());

  vis_pub_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  RMPMeshPlanner rmp_planner;
  OMPLMeshSamplingPlanner ompl_planner(false);
  OMPLMeshSamplingPlanner ompl_planner_connect(true);

  while (ros::ok()) {
    std::vector<cad_percept::cgal::Point> points;
    std::copy_n(*rng_mesh_, 3, std::back_inserter(points));
    Eigen::Vector3d startpoint, endpoint;
    startpoint.x() = points[1].x();
    startpoint.y() = points[1].y();
    startpoint.z() = points[1].z();
    endpoint.x() = points[2].x();
    endpoint.y() = points[2].y();
    endpoint.z() = points[2].z();

    /*  startpoint.x() = 1.568;
      startpoint.y() = 3.6077;
      startpoint.z() = 4.29022;
      endpoint.x() = 17.1972;
      endpoint.y() = 5.37636;
      endpoint.z() = 5.13425;*/

    std::vector<Eigen::Vector3d> path_rrt_star;

    std::chrono::steady_clock::time_point begin_rrt_star = std::chrono::steady_clock::now();
    bool rrt_star_success = ompl_planner.plan(startpoint, endpoint, &path_rrt_star);
    std::chrono::steady_clock::time_point end_rrt_star = std::chrono::steady_clock::now();

    visualizePath(path_rrt_star, {0.0, 1.0, 0.0}, 0);

    std::vector<Eigen::Vector3d> path_rrt_connect;
    std::chrono::steady_clock::time_point begin_rrt_connect = std::chrono::steady_clock::now();

    bool rrt_connect_success = ompl_planner_connect.plan(startpoint, endpoint, &path_rrt_connect);
    std::chrono::steady_clock::time_point end_rrt_connect = std::chrono::steady_clock::now();

    visualizePath(path_rrt_connect, {0.0, 0.0, 1.0}, 1);

    std::vector<Eigen::Vector3d> path_rmp;
    std::chrono::steady_clock::time_point begin_rmp = std::chrono::steady_clock::now();
    bool rmp_success =  rmp_planner.plan(startpoint, endpoint, &path_rmp);
    std::chrono::steady_clock::time_point end_rmp = std::chrono::steady_clock::now();
    visualizePath(path_rmp, {1.0, 0.0, 0.0}, 2);

    std::cout << "RRT*\t =\t "
              << std::chrono::duration_cast<std::chrono::microseconds>(end_rrt_star -
                                                                       begin_rrt_star)
                     .count()
              << "[µs]" << std::endl;
    std::cout << "BiRRT\t =\t "
              << std::chrono::duration_cast<std::chrono::microseconds>(end_rrt_connect -
                                                                       begin_rrt_connect)
                     .count()
              << "[µs]" << std::endl;
    std::cout << "RMP\t =\t "
              << std::chrono::duration_cast<std::chrono::microseconds>(end_rmp - begin_rmp).count()
              << "[µs]" << std::endl;

    std::cout << "Data\t"
              << (rrt_star_success ? std::chrono::duration_cast<std::chrono::microseconds>(end_rrt_star -
                                                                       begin_rrt_star)
                     .count() : -1)
              << "\t"
              << (rrt_connect_success ? std::chrono::duration_cast<std::chrono::microseconds>(end_rrt_connect -
                                                                       begin_rrt_connect)
                     .count() : -1)
              << "\t"
              << (rmp_success ? std::chrono::duration_cast<std::chrono::microseconds>(end_rmp - begin_rmp).count() : -1)
              << std::endl;
  }
  return 0;
}