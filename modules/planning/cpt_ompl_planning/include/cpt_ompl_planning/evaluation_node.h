#ifndef CPT_OMPL_PLANNING_EVALUATION_NODE_H
#define CPT_OMPL_PLANNING_EVALUATION_NODE_H
#include <CGAL/point_generators_3.h>
#include <cpt_ompl_planning/ompl_mesh_sampling_planner.h>
#include <cpt_ompl_planning/path_kpi_calculator.h>
#include <cpt_planning/implementation/geodesic_mesh_planner.h>
#include <cpt_planning/implementation/rmp_mesh_planner.h>
#include <cpt_planning/interface/surface_planner.h>
#include <cpt_ros/mesh_model_publisher.h>

#include <chrono>
#include <vector>
using cad_percept::planning::SurfacePlanner;

class EvaluationNode {
  typedef struct {
    int id;
    SurfacePlanner::Ptr instance;
    Eigen::Vector3d color;
  } PlannerEvaluation;

 public:
  EvaluationNode(ros::NodeHandle nh, std::string mesh_path) : nh_(nh) {
    cad_percept::cgal::MeshModel::create(mesh_path, &model_, true);
    pub_model_ = cad_percept::MeshModelPublisher(nh_, "mesh");
    random_sampler_ =
        std::make_shared<CGAL::Random_points_in_triangle_mesh_3<cad_percept::cgal::Polyhedron>>(
            model_->getMeshRef());
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    pub_model_.publish(model_);
    kpi_calculator_.setModel(model_);
  }

  void addPlanner(SurfacePlanner::Ptr planner, Eigen::Vector3d color) {
    planners_.push_back({last_id_++, planner, color});
  }

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
    marker.scale.x = 0.05;
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

    pub_marker_.publish(marker);
  }

  void visualizeEdgeList(SurfacePlanner::EdgeList list, Eigen::Vector3d color, std::string name) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "edgelist " + name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;
    marker.color.a = 0.45;  // Don't forget to set the alpha!
    marker.color.r = color.x();
    marker.color.g = color.y();
    marker.color.b = color.z();

    for (auto edge : list) {
      geometry_msgs::Point pt_start;
      pt_start.x = edge.first.x();
      pt_start.y = edge.first.y();
      pt_start.z = edge.first.z();
      marker.points.push_back(pt_start);

      geometry_msgs::Point pt_end;
      pt_end.x = edge.second.x();
      pt_end.y = edge.second.y();
      pt_end.z = edge.second.z();
      marker.points.push_back(pt_end);
    }

    pub_marker_.publish(marker);
  }

  void logResult(int id, SurfacePlanner::Result result, std::string name,
                 std::vector<Eigen::Vector3d> path) {
    std::cout << "DATA\t" << name << "\t =\t " << result.success << "\t"
              << std::chrono::duration_cast<std::chrono::microseconds>(result.duration).count()
              << "[µs]" << std::endl;
  }

  void getRandomPos(Eigen::Vector3d* pos_out) {
    std::vector<cad_percept::cgal::Point> points;

    // we always sample 2 points - somehow we don't get a new point when only sampling 1
    std::copy_n(*random_sampler_, 2, std::back_inserter(points));
    pos_out->x() = points[1].x();
    pos_out->y() = points[1].y();
    pos_out->z() = points[1].z();
  }

  void plan(bool new_random = true) {
    if (new_random) {
      getRandomPos(&start_);
      getRandomPos(&goal_);
    }

    for (auto planner : planners_) {
      std::vector<Eigen::Vector3d> path;

      // run planner
      auto result = planner.instance->plan(start_, goal_, &path);
      logResult(planner.id, result, planner.instance->getName(), path);
      if (result.success) {
        visualizePath(path, planner.color, planner.id);
      }
      auto edgelist = planner.instance->getEdges();
      if (edgelist.size() > 0) {
        visualizeEdgeList(edgelist, planner.color, planner.instance->getName());
      }
      std::cout << std::endl;
      std::cout << "------------------" << std::endl;
      std::cout << planner.instance->getName() << std::endl;
      std::cout << "------------------" << std::endl;
      std::cout << kpi_calculator_.calculate(path);
      std::cout << "------------------" << std::endl;
      std::cout << std::endl;
    }
  }
  Eigen::Vector3d start_, goal_;
  int last_id_{0};
  ros::NodeHandle nh_;
  std::vector<PlannerEvaluation> planners_;
  cad_percept::cgal::MeshModel::Ptr model_;

  std::shared_ptr<CGAL::Random_points_in_triangle_mesh_3<cad_percept::cgal::Polyhedron>>
      random_sampler_;

  cad_percept::MeshModelPublisher pub_model_;
  ros::Publisher pub_marker_;
  PathKPICalcuator kpi_calculator_;
};

#endif  // CPT_OMPL_PLANNING_EVALUATION_NODE_H
