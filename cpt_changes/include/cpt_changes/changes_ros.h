#ifndef CHANGES_ROS_H_
#define CHANGES_ROS_H_

#include <kindr/minimal/quat-transformation-gtsam.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <shape_msgs/Mesh.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <cgal_msgs/ProbabilisticMesh.h>
#include "cgal_conversions/mesh_conversions.h"
#include "cgal_definitions/mesh_model.h"
#include "cpt_utils/cpt_utils.h"

namespace cad_percept {
namespace changes {

typedef kindr::minimal::QuatTransformationTemplate<double> SE3;
typedef SE3::Rotation SO3;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class ChangesRos {
	public:
		ChangesRos(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
		~ChangesRos();

		// Associate point-cloud with architect model.
		void associatePointCloud(const PointCloud &pc_msg);

		// Publishing of colorized association markers
		void publishColorizedAssocMarkers(const cpt_utils::Associations &associations);

		// Service call to transform the architect model.
		bool transformModelCb(std_srvs::Empty::Request &request,
													std_srvs::Empty::Response &response);

		// Publishing of architect model as point cloud.
		void publishArchitectModel() const;

		// Publishing of architect model as mesh
		void publishArchitectModelMesh() const;

		// Publishing of colorized association triangles
		void publishColorizedAssocTriangles(const cpt_utils::Associations associations) const;

	private:
		ros::NodeHandle &nh_, nh_private_;
		cgal::MeshModel mesh_model_;
		ros::Publisher good_matches_pub_, bad_matches_pub_, model_pub_, arch_pub_, mesh_pub_, distance_triangles_pub_;
		ros::Subscriber pointcloud_sub_;
		visualization_msgs::Marker model_;
		ros::ServiceServer transformSrv_;
		tf::TransformListener tf_listener_;
		std::string map_frame_, cad_frame_;
		double distance_threshold_;
		bool discrete_color_;

};     

}
}

#endif // CHANGES_ROS_H_
