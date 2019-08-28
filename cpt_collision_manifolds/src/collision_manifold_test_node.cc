#include <cpt_collision_manifolds/iso_surface_manifold.h>
#include <cgal_msgs/TriangleMesh.h>
#include <ros/ros.h>

namespace cad_percept {
namespace cpt_collision_manifolds {

class CollisionManifoldTestNode {
 public:
  CollisionManifoldTestNode(ros::NodeHandle& nh, ros::NodeHandle nh_private) :
      nh_(nh),
      nh_private_(nh_private) {

    // Create simple subscriber for a mesh.
    nh_.subscribe("mesh", 1,
                  &CollisionManifoldTestNode::meshCallback, this);

    // Publisher for the constructed manifold.
    pub_collision_manifold_ = nh_private_.advertise<cgal_msgs::TriangleMesh>(
        "collision_manifold", 1);

    // Create iso surface manifold w radius 0.3 m
    collision_manifold_(std::make_shared<IsoSurfaceManifold>(0.3));
  }

 private:
  void meshCallback(const cgal_msgs::TriangleMeshConstPtr& mesh) {

  }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pub_collision_manifold_;

  std::shared_ptr<CollisionManifoldInterface> collision_manifold_;

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "collision_manifold_test_node");
  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create Node object
  CollisionManifoldTestNode node(nh, nh_private);

  ros::spin();

  return 0;
}

}
}