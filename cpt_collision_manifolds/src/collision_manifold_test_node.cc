#include <cgal_conversions/mesh_conversions.h>
#include <cgal_msgs/TriangleMesh.h>
#include <cpt_collision_manifolds/iso_surface_manifold.h>
#include <ros/ros.h>

namespace cad_percept {
namespace collision_manifolds {

/*
 * For now we leave the Testnode inside this file, it will be factored out
 * later into a ros-specific package.
 */
class CollisionManifoldTestNode {
 public:
  CollisionManifoldTestNode(ros::NodeHandle& nh, ros::NodeHandle nh_private)
      : nh_(nh), nh_private_(nh_private) {
    // Create simple subscriber for a mesh.
    nh_.subscribe("mesh", 1, &CollisionManifoldTestNode::meshCallback, this);

    // Publisher for the constructed manifold.
    pub_collision_manifold_ =
        nh_private_.advertise<cgal_msgs::TriangleMesh>("collision_manifold", 1);
  }

 private:
  void meshCallback(const cgal_msgs::TriangleMeshConstPtr& mesh) {
    cgal::PolyhedronPtr original_surface = std::make_shared<cgal::Polyhedron>();
    cgal::msgToTriangleMesh(*mesh, original_surface.get());

    // Create construction strategy
    auto construction_strategy = std::make_shared<offset_surface::VertexNormalStrategy>();

    // Create collision manifold w radius 0.3
    IsoSurfaceManifold collision_manifold(
        original_surface, 0.3,
        std::dynamic_pointer_cast<offset_surface::ConstructionStrategy>(construction_strategy));

    // Get manifold back as mesh
    cgal::PolyhedronPtr collision_manifold_mesh = std::make_shared<cgal::Polyhedron>();
    collision_manifold.getAsMesh(collision_manifold_mesh.get());

    // send message with collision manifold
    cgal_msgs::TriangleMesh output_msg;
    cgal::triangleMeshToMsg(*collision_manifold_mesh, &output_msg);
    pub_collision_manifold_.publish(output_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pub_collision_manifold_;
};

}  // namespace collision_manifolds
}  // namespace cad_percept

int main(int argc, char** argv) {
  ros::init(argc, argv, "collision_manifold_test_node");
  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create Node object
  cad_percept::collision_manifolds::CollisionManifoldTestNode node(nh, nh_private);

  ros::spin();

  return 0;
}