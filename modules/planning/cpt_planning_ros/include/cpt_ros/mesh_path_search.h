
#ifndef CPT_ROS_INCLUDE_CPT_ROS_MESH_PATH_SEARCH_H_
#define CPT_ROS_INCLUDE_CPT_ROS_MESH_PATH_SEARCH_H_
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/mesh_model.h>
#include <ros/ros.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <Eigen/Dense>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Ray_3 Ray;
typedef K::Point_3 Point;
typedef CGAL::Polyhedron_3<K, CGAL::Polyhedron_items_with_id_3> Polyhedron;
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
typedef CGAL::AABB_traits<K, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;

namespace cad_percept {

class MeshPathSearch {
 public:
  MeshPathSearch(){}

  MeshPathSearch( cgal::MeshModel::Ptr model)
      : poly_(model->getMesh()), mesh_model_(model) {}

//   void publish(cgal::MeshModel::Ptr model, std::string frame="world") {
//     cgal_msgs::TriangleMeshStamped msg;
//     cgal::Polyhedron poly(model->getMesh());
//     cgal::triangleMeshToMsg(poly, &msg.mesh);
//     msg.header.stamp = ros::Time::now();
//     msg.header.frame_id = "world";

//     if(frame == "uv"){
//       for(auto& vertex : msg.mesh.vertices){
//         vertex.x += 2.0;
//       }
//     }
//     mesh_publisher_.publish(msg);
//   }

  int rayMeshIntersection(Point ray_a, Point ray_b){
    // constructs AABB tree
    Tree tree(faces(poly_).first, faces(poly_).second, poly_);
    // counts #intersections
    Ray ray_query(ray_a,ray_b);
    int num = tree.number_of_intersected_primitives(ray_query);
    std::cout << num
        << " intersections(s) with ray query" << std::endl;
    return num;
  }

  Point rayGenAngleAxis(Point init_p, Point target_p, double angle, Eigen::Vector3d axis){
    Eigen::AngleAxisd angleAxis(angle, axis);
    Eigen::Vector3d vec(target_p.x()-init_p.x(),
                        target_p.y()-init_p.y(),
                        target_p.z()-init_p.z());
    Eigen::Vector3d rot_vec = angleAxis.matrix()*vec;
    Point p(init_p.x()+rot_vec.x(),
                  init_p.y()+rot_vec.y(),
                  init_p.z()+rot_vec.z());
    return p;
  }

  void rayScan(Point init_p, Point target_p, double interval_angle, Eigen::Vector3d axis){
    Point new_target = target_p;
    for(double angle=0; angle<=2*M_PI; angle += interval_angle){
      rayMeshIntersection(init_p, new_target);
      new_target = rayGenAngleAxis(init_p, target_p, angle, axis);
    }
  }

//weighted metrics
  Eigen::Vector3d meshToRopeVec(
    std::vector<Eigen::Vector3d> rope, int start_idx, int end_idx,
    double node_dist, double safe_dist){
    //find the closest rope to mesh face vector
    Eigen::Vector3d min_repulsion_vec;
    Eigen::Vector3d repulsion_cost = Eigen::Vector3d::Zero();
    int min_node_id;
    for(int node_id = start_idx; node_id<end_idx; node_id++){
      Eigen::Vector3d node = rope.at(node_id);
      cgal::PointAndPrimitiveId ppid = mesh_model_->getClosestTriangle(node.x(),node.y(),node.z());
      Eigen::Vector3d p_on_mesh(ppid.first.x(),ppid.first.y(),ppid.first.z());
      Eigen::Vector3d repulsion_vec = node-p_on_mesh;
      if(start_idx == 0 && 
        repulsion_vec.norm()+(node_id-start_idx)*node_dist < safe_dist){
        continue;
      }
      repulsion_cost = repulsion_cost + repulsion_vec.normalized()*(1/pow(repulsion_vec.norm(),3));
    }
    repulsion_cost = repulsion_cost.normalized()*repulsion_cost.norm()/rope.size();
    return repulsion_cost;
  }


 private:
  // ros::NodeHandle nh_; 
  // ros::Publisher mesh_publisher_;
  // std::string topic_name_;
  Polyhedron poly_;
  cgal::MeshModel::Ptr mesh_model_;

};
}  // namespace cad_percept
#endif  // CPT_ROS_INCLUDE_CPT_ROS_MESH_PATH_SEARCH_H_
