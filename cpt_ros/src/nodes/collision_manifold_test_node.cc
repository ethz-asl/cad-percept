#include <cgal_conversions/mesh_conversions.h>
#include <cgal_msgs/TriangleMesh.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_collision_manifolds/iso_surface_manifold.h>
#include <cpt_collision_manifolds/offset_surface/meshdomain_strategy.h>
#include <cpt_ros/ros_config_provider.h>
#include <cpt_utils/perf.h>
#include <ros/ros.h>

namespace cad_percept {

/*
 * For now we leave the Testnode inside this file, it will be factored out
 * later into a ros-specific package.
 */
class CollisionManifoldTestNode {
 public:
  CollisionManifoldTestNode(ros::NodeHandle& nh, ros::NodeHandle nh_private)
      : nh_(nh), nh_private_(nh_private) {
    // Create simple subscriber for a mesh.
    sub_surface_manifold_ =
        nh_private_.subscribe("mesh", 1, &CollisionManifoldTestNode::meshCallback, this);

    // Publisher for the constructed manifold.
    pub_collision_manifold_ =
        nh_private_.advertise<cgal_msgs::TriangleMeshStamped>("collision_manifold", 1);

    // valid strategies: meshdomain, vertexnormal
    const std::string offset_strategy =
        nh_private_.param<std::string>("offset_strategy/type", "vertexnormal");

    // set up config provider in subnamespace
    ConfigProvider::Ptr cfg(
        std::make_shared<RosConfigProvider>(ros::NodeHandle(nh_private_, "offset_strategy")));

    // set up construction strategy
    // TODO(mpantic): At some point replace by a nice factory.
    if (offset_strategy == "meshdomain") {
      surface_construct_ =
          std::make_shared<collision_manifolds::offset_surface::MeshDomainStrategy>(cfg);
    } else {
      // we assume "vertexnormal"
      surface_construct_ =
          std::make_shared<collision_manifolds::offset_surface::VertexNormalStrategy>(cfg);
    }
  }

 private:
  void meshCallback(const cgal_msgs::TriangleMeshStampedConstPtr& mesh) {
    ROS_INFO_STREAM("Mesh Received w/ " << mesh->mesh.vertices.size() << " vertices.");
    cgal::PolyhedronPtr original_surface = std::make_shared<cgal::Polyhedron>();
    cgal::msgToTriangleMesh(mesh->mesh, original_surface.get());

    // Create collision manifold
    double offset_distance = nh_private_.param("offset_distance", 0.1);
    collision_manifolds::IsoSurfaceManifold collision_manifold(original_surface, offset_distance,
                                                               surface_construct_);
    collision_manifold.construct();

    // Get manifold back as mesh
    cgal::PolyhedronPtr collision_manifold_mesh = std::make_shared<cgal::Polyhedron>();
    collision_manifold.getAsMesh(collision_manifold_mesh.get());

    // send message with collision manifold
    cgal_msgs::TriangleMeshStamped output_msg;
    output_msg.header = mesh->header;
    cgal::triangleMeshToMsg(*collision_manifold_mesh, &output_msg.mesh);
    pub_collision_manifold_.publish(output_msg);
    ROS_INFO_STREAM("  - CM published w/ " << output_msg.mesh.vertices.size() << " vertices.");
    Perf::get()->printStatistics();
  }

  collision_manifolds::offset_surface::ConstructionStrategy::Ptr surface_construct_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pub_collision_manifold_;
  ros::Subscriber sub_surface_manifold_;
};

}  // namespace cad_percept

int main(int argc, char** argv) {
  ros::init(argc, argv, "collision_manifold_test_node");
  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Create Node object
  cad_percept::CollisionManifoldTestNode node(nh, nh_private);

  ros::spin();

  return 0;
}
