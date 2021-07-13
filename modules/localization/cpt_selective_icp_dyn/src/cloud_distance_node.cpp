#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <cgal_conversions/eigen_conversions.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_msgs/ColoredMesh.h>
#include <cgal_msgs/ReferenceTask.h>
#include <cgal_msgs/TriangleMesh.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_utils/pc_processing.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/point_types.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/Timer.h>
#include <pointmatcher_ros/point_cloud.h>
#include <pointmatcher_ros/transform.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/thread.hpp>
#include <unordered_set>
//#include "cpt_selective_icp_dyn/BuildingTask.h"
//#include "cpt_selective_icp_dyn/References.h"
#include "cpt_selective_icp_dyn/utils.h"

// parameters
std::string cad_topic;
std::string cloud_in_topic;
std::string cloud_out_topic;
std::string tf_map_frame;
std::string cloud_frame;
double dist_threshold;
int input_queue_size;
bool cad_trigger;
int map_sampling_density;
bool ref_mesh_ready;

namespace cad_percept {
namespace selective_icp_dyn {

class CloudDistance
{
  private:
    tf::TransformListener tf_listener_;
    ros::Subscriber cad_sub_;
    ros::Subscriber cloud_in_sub_;
    ros::Publisher cloud_out_pub_;
    ros::ServiceServer load_published_map_srv_;
    // transformation
    std::shared_ptr<PM::Transformation> transformation_ = PM::get().REG(Transformation).create("RigidTransformation");
    // for reference frame
    cgal::MeshModel::Ptr reference_mesh_;
    PM::TransformationParameters T_map_to_meshorigin_;
    std::string mesh_frame_id_;
    PM::TransformationParameters T_scanner_to_map_;

    // Callback for initializing mesh
    void gotCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in) {
      if (cad_trigger) {
        std::cout << "Processing CAD mesh" << std::endl;
        mesh_frame_id_ = cad_mesh_in.header.frame_id;  // should be "marker2"
        cgal::msgToMeshModel(cad_mesh_in.mesh, &reference_mesh_);

        // Make some checks:
        cgal::Polyhedron P = reference_mesh_->getMesh();
        if (P.is_valid()) {
          std::cout << "P is valid" << std::endl;
        } else {
          std::cerr << "P is not valid" << std::endl;
        }
        if (P.is_pure_triangle()) {
          std::cout << "P is pure triangle" << std::endl;
        } else {
          std::cerr << "P is not pure triangle" << std::endl;
        }
        if (P.is_closed()) {
          std::cout << "P is closed" << std::endl;
        } else {
          std::cerr << "P is not closed => no consistent full directions" << std::endl;
        }

        tf::StampedTransform transform;
        tf_listener_.lookupTransform(tf_map_frame, mesh_frame_id_, ros::Time(0),
                                     transform);  // from origin of cad model to map
        Eigen::Matrix3d rotation;
        tf::matrixTFToEigen(transform.getBasis(), rotation);
        Eigen::Vector3d translation;
        tf::vectorTFToEigen(transform.getOrigin(), translation);
        Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
        transformation.block(0, 0, 3, 3) = rotation;
        transformation.block(0, 3, 3, 1) = translation;
        cgal::Transformation ctransformation;
        cgal::eigenTransformationToCgalTransformation(transformation, &ctransformation);
        reference_mesh_->transform(ctransformation);

        // save the transform map -> mesh origin (inverse) for later reference
        Eigen::Affine3d eigenTr;
        tf::transformTFToEigen(transform.inverse(), eigenTr);
        T_map_to_meshorigin_ = eigenTr.matrix().cast<float>();

        ref_mesh_ready = true;

        ROS_INFO("Ending gotCAD");
      }
    }
    
    void gotCloud(const sensor_msgs::PointCloud2 &cloud_msg_in) {
      if (ref_mesh_ready) {
        // Get original pointcloud
        const ros::Time stamp = cloud_msg_in.header.stamp;
        DP originalCloud(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloud_msg_in));
        const int dimp1(originalCloud.features.rows());
        // Get transform

        try {
          T_scanner_to_map_ = PointMatcher_ros::eigenMatrixToDim<float>(
              PointMatcher_ros::transformListenerToEigenMatrix<float>(tf_listener_,
                                                                      tf_map_frame,  // to
                                                                      cloud_frame,   // from
                                                                      stamp),
              dimp1);
        } catch (tf::ExtrapolationException e) {
          ROS_ERROR_STREAM("Extrapolation Exception. stamp = "
                           << stamp << " now = " << ros::Time::now()
                           << " delta = " << ros::Time::now() - stamp << std::endl
                           << e.what());
          return;
        } catch (...) {
          // Everything else.
          ROS_ERROR_STREAM("Unexpected exception... ignoring scan.");
          return;
        }

        
        // Transform pointcloud
        DP pc_original = transformation_->compute(originalCloud, T_scanner_to_map_);
        publishDistanceToMeshAsPC(pc_original, cloud_out_pub_, stamp);
        std::cout << "published pc with distance information" << std::endl;
      }
      else{
        std::cout << "cad not yet initialized!";
      }
    }

    bool loadPublishedMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
      // Since CAD is published all the time, we need a trigger when to load it
      cad_trigger = true;
      return true;
    }

    void publishDistanceToMeshAsPC(const DP &aligned_cloud, const ros::Publisher &pub,
                                   const ros::Time &stamp) {
      // Convert DP to PC
      PointCloud aligned_pc = utils::dpToPointCloud(aligned_cloud);

      // 5 attributes in PC
      const int dimFeatures = 5;

      PM::Matrix feat(dimFeatures, aligned_pc.points.size());

      // Calculate distance to closes triangle for each point of the pointcloud
      for (uint i = 0; i < aligned_pc.points.size(); ++i) {
        cgal::PointAndPrimitiveId ppid =
            reference_mesh_->getClosestTriangle(aligned_pc[i].x, aligned_pc[i].y, aligned_pc[i].z);
        float squared_distance = (float)sqrt(reference_mesh_->squaredDistance(
            cgal::Point(aligned_pc[i].x, aligned_pc[i].y, aligned_pc[i].z)));
        feat(0, i) = aligned_pc[i].x;
        feat(1, i) = aligned_pc[i].y;
        feat(2, i) = aligned_pc[i].z;
        feat(3, i) = squared_distance;
        feat(4, i) = 1.0;
      }

      DP::Labels featLabels;
      featLabels.push_back(DP::Label("x", 1));
      featLabels.push_back(DP::Label("y", 1));
      featLabels.push_back(DP::Label("z", 1));
      featLabels.push_back(DP::Label("intensity", 1));
      featLabels.push_back(DP::Label("pad", 1));

      // Create pointcloud from matrix
      DP dppointcloud = DP(feat, featLabels);

      // publish pointcloud
      pub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
          dppointcloud, tf_map_frame, stamp));
    }

   public:
    CloudDistance(ros::NodeHandle &nh_) {
      cad_sub_ = nh_.subscribe(cad_topic, 1, &CloudDistance::gotCAD, this);
      cloud_in_sub_ = nh_.subscribe(cloud_in_topic, 1, &CloudDistance::gotCloud, this);
      cloud_out_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_out_topic, 50, true);
      load_published_map_srv_ =
      nh_.advertiseService("load_published_map", &CloudDistance::loadPublishedMap, this);
    };
};
}  // namespace selective_icp_dyn
}  // namespace cad_percept

int main(int argc, char **argv) {
  ros::init(argc, argv, "cloud_distance");
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private("~");

  // Parameters
  ros::NodeHandle("~").getParam("cadTopic", cad_topic);
  ros::NodeHandle("~").getParam("cloudInTopic", cloud_in_topic);
  ros::NodeHandle("~").getParam("inputQueueSize", input_queue_size);
  ros::NodeHandle("~").getParam("cloudOutTopic", cloud_out_topic);
  ros::NodeHandle("~").getParam("tfMapFrame", tf_map_frame);
  ros::NodeHandle("~").getParam("cloudFrame", cloud_frame);
  ros::NodeHandle("~").getParam("mapSamplingDensity", map_sampling_density);
  ros::NodeHandle("~").getParam("distThreshold", dist_threshold);
  cad_trigger = false;
  ref_mesh_ready = false;

  cad_percept::selective_icp_dyn::CloudDistance clouddistance(nh_);

  std::cout << "Wait for start-up" << std::endl;
  sleep(5);  // wait to set up stuff
  std::cout << "Ready!" << std::endl;
  ros::spin();

  return 0;
};

