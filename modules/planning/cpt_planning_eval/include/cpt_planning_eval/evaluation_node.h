#ifndef CPT_PLANNING_EVAL_EVALUATION_NODE_H
#define CPT_PLANNING_EVAL_EVALUATION_NODE_H
#include <CGAL/point_generators_3.h>
#include <cpt_planning/interface/surface_planner.h>
#include <cpt_planning_eval/tools/path_kpi_calculator.h>
#include <cpt_ros/mesh_model_publisher.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <chrono>
#include <fstream>
#include <vector>

namespace cad_percept {
namespace planning {

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
                 std::vector<Eigen::Vector3d> path, PathKPICalcuator::QualityIndicator quality,
                 std::string run_id) {
    std::cout << "DATA\t" << name << "\t" << result.success << "\t"
              << std::chrono::duration_cast<std::chrono::microseconds>(result.duration).count()
              << "\t" << quality.length << "\t" << quality.surface_dist.avg << "\t"
              << quality.smoothness.avg << "\t" << quality.segments << "\t" << rowid << "\t"
              << run_id << std::endl;
  }

  void logPath(std::string run_id, std::vector<Eigen::Vector3d> path) {
    std::ofstream logfile;
    logfile.open("path_" + run_id + ".log");
    for (auto node : path) {
      logfile << node.x() << "\t" << node.y() << "\t" << node.z() << std::endl;
    }
    logfile.close();
  }

  void logEdges(std::string run_id, SurfacePlanner::EdgeList list) {
    std::ofstream logfile;
    logfile.open("edges_" + run_id + ".log");
    for (auto edge : list) {
      logfile << edge.first.x() << "\t" << edge.first.y() << "\t" << edge.first.z()
              << edge.second.x() << "\t" << edge.second.y() << "\t" << edge.second.z() << std::endl;
    }
    logfile.close();
  }

  void getRandomPos(Eigen::Vector3d* pos_out) {
    std::vector<cad_percept::cgal::Point> points;

    // we always sample 2 points - somehow we don't get a new point when only sampling 1
    std::copy_n(*random_sampler_, 2, std::back_inserter(points));
    pos_out->x() = points[1].x();
    pos_out->y() = points[1].y();
    pos_out->z() = points[1].z();
  }

  void plan(bool new_random = true, bool write = true, Eigen::Vector3d start = {0, 0, 0},
            Eigen::Vector3d goal = {0, 0, 0}) {
    if (new_random) {
      getRandomPos(&start_);
      getRandomPos(&goal_);
    } else {
      start_ = start;
      goal_ = goal;
    }
    std::cout << start_ << std::endl;
    std::cout << goal_ << std::endl;

    for (auto planner : planners_) {
      std::vector<Eigen::Vector3d> path;

      // run planner
      // create id for this run
      std::string run_id = boost::uuids::to_string(uuid_gen_());

      auto result = planner.instance->plan(start_, goal_, &path);
      auto kpis = kpi_calculator_.calculate(path);
      logResult(planner.id, result, planner.instance->getName(), path, kpis, run_id);

      if (result.success) {
        visualizePath(path, planner.color, planner.id);
        if (write) {
          logPath(run_id, path);
        }
      }
      auto edgelist = planner.instance->getEdges();
      if (edgelist.size() > 0) {
        visualizeEdgeList(edgelist, planner.color, planner.instance->getName());
        logEdges(run_id, edgelist);
      }
    }
    rowid++;
  }

  int rowid{0};
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
  boost::uuids::random_generator uuid_gen_;
};
}  // namespace planning
}  // namespace cad_percept
#endif  // CPT_PLANNING_EVAL_EVALUATION_NODE_H
