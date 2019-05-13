#include "cpt_changes/changes_ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <tf_conversions/tf_eigen.h>

namespace cad_percept {
namespace changes {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ChangesRos::ChangesRos(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
	: nh_(nh),
		nh_private_(nh_private),
		mesh_model_(nh_private.param<std::string>("off_model", "fail")),
		map_frame_(nh_private.param<std::string>("map_frame", "fail")),
		cad_frame_(nh_private.param<std::string>("cad_frame", "fail")),
		distance_threshold_(nh_private.param<double>("distance_threshold", 0.2)),
		discrete_color_(nh_private.param<bool>("discrete_color", false)){
	if (!nh_private_.hasParam("off_model"))
			std::cerr << "ERROR 'off_model' not set as parameter." << std::endl;
	good_matches_pub_ =
			nh_.advertise<visualization_msgs::Marker>("good_cad_matches", 100);
	bad_matches_pub_ =
			nh_.advertise<visualization_msgs::Marker>("bad_cad_matches", 100);
	model_pub_ =
			nh_.advertise<visualization_msgs::Marker>("architect_model", 100);
	arch_pub_ =
			nh_.advertise<PointCloud>("architect_model_pcl", 100, true); // latching to true (saves last messages and sends to future subscriber)
	mesh_pub_ = 
			nh_.advertise<cgal_msgs::ProbabilisticMesh>("mesh_model", 100, true); // latching to true
	distance_triangles_pub_ =
			nh_.advertise<cgal_msgs::ProbabilisticMesh>("distance_mesh", 100, true);

	pointcloud_sub_ = nh_private_.subscribe(nh_private.param<std::string>("scan_topic", "fail"), 10, &ChangesRos::associatePointCloud, this);
	
	// start defining visualization_msgs Marker, this is just a block, right?
	model_.type = visualization_msgs::Marker::MESH_RESOURCE;
	model_.ns = "primitive_surfaces";
	if (!nh_private_.hasParam("stl_model"))
		std::cerr << "ERROR 'stl_model' not set as parameter." << std::endl;
	model_.mesh_resource =
			"file://" + nh_private_.param<std::string>("stl_model", "fail");
	model_.header.frame_id = map_frame_;
	model_.scale.x = 1.0;
	model_.scale.y = 1.0;
	model_.scale.z = 1.0;
	model_.pose.position.x = 0;
	model_.pose.position.y = 0;
	model_.pose.position.z = 0;
	model_.pose.orientation.x = 0.0;
	model_.pose.orientation.y = 0.0;
	model_.pose.orientation.z = 0.0;
	model_.pose.orientation.w = 1.0;
	model_.color.a = 1.0;
	model_.color.r = 1.0;
	model_.color.g = 0.0f;
	model_.color.b = 0.0f;

	transformSrv_ =
			nh.advertiseService("transformModel", &ChangesRos::transformModelCb, this);
}

ChangesRos::~ChangesRos() {}

// callback for each p.c. scan topic
void ChangesRos::associatePointCloud(const PointCloud &pc_msg) {
	cpt_utils::Associations associations = cpt_utils::associatePointCloud(pc_msg, &mesh_model_);
	CHECK_EQ(associations.points_from.cols(), pc_msg.width); // glog, checks equality
	CHECK_EQ(associations.points_to.cols(), pc_msg.width);
	CHECK_EQ(associations.distances.rows(), pc_msg.width);
	CHECK_EQ(associations.triangles_to.rows(), pc_msg.width);

	publishArchitectModelMesh();
	publishArchitectModel();
	publishColorizedAssocMarkers(associations);
	publishColorizedAssocTriangles(associations);

	// todo: Filter out points that are close to several (non-parallel) planes.
	// -> because they might be associated to the wrong plane
}

void ChangesRos::publishColorizedAssocMarkers(const cpt_utils::Associations &associations) {
	visualization_msgs::Marker good_marker, bad_marker;
	good_marker.header.frame_id = map_frame_;
	good_marker.ns = "semantic_graph_matches";
	good_marker.type = visualization_msgs::Marker::LINE_LIST;
	good_marker.action = visualization_msgs::Marker::ADD;
	good_marker.id = 0;

	good_marker.scale.x = 0.01f;
	good_marker.color.r = 0.0f;
	good_marker.color.g = 1.0f;
	good_marker.color.b = 0.0f;
	good_marker.color.a = 0.7f;
	bad_marker = good_marker;
	bad_marker.color.r = 1.0f;
	bad_marker.color.g = 0.0f;

	geometry_msgs::Point p_from, p_to;
	int differences = 0; // number of differences in current reading
	// for each point i in point cloud:
	for (uint i = 0; i < associations.points_from.cols(); ++i) {

	// todo: less copying
		p_from.x = associations.points_from(0, i); // x
		p_from.y = associations.points_from(1, i); // y
		p_from.z = associations.points_from(2, i); // z
		p_to.x = associations.points_to(0, i); // x
		p_to.y = associations.points_to(1, i); // y
		p_to.z = associations.points_to(2, i); // z
		double length = associations.distances(i);
		if (length > distance_threshold_) {
			differences++;
			bad_marker.points.push_back(p_from); // marker makes point between these two
			bad_marker.points.push_back(p_to);
		} else {
			good_marker.points.push_back(p_from);
			good_marker.points.push_back(p_to);
		}
	}
	std::cout << "number of differences in current reading: " << differences << "/" << associations.points_from.cols()
			<< std::endl;

	good_matches_pub_.publish(good_marker);
	bad_matches_pub_.publish(bad_marker);
	model_pub_.publish(model_); // what is this?
}

// how to handle empty request, response?
bool ChangesRos::transformModelCb(std_srvs::Empty::Request &request,
                                std_srvs::Empty::Response &response) {
	tf::StampedTransform transform; //tf is ROS package for coordinate frames
	tf_listener_.lookupTransform(map_frame_, cad_frame_, ros::Time(0), transform); // transform from cad_frame_ to map_frame_, latest available transform, but where is cad_frame?
	Eigen::Matrix3d rotation;
	tf::matrixTFToEigen(transform.getBasis(), rotation); // basis matrix for rotation
	Eigen::Vector3d translation;
	tf::vectorTFToEigen(transform.getOrigin(), translation); // origin vector translation
	Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
	transformation.block(0, 0, 3, 3) = rotation;
	transformation.block(0, 3, 3, 1) = translation;
	mesh_model_.transform(cgal::eigenTransformationToCgalTransformation(transformation)); // apply transform from cad_frame_ to map_frame_ to get aligned data
	return true;
}

// publishes architect model mesh as point cloud of vertices
void ChangesRos::publishArchitectModel() const {
	PointCloud pc_msg;
	cgal::meshToVerticePointCloud(mesh_model_.getMesh(), &pc_msg);
	pc_msg.header.frame_id = map_frame_;
	pcl_conversions::toPCL(ros::Time(0), pc_msg.header.stamp); // ROS time stamp to PCL time stamp
	arch_pub_.publish(pc_msg);
}

// publish architect model mesh as triangle mesh
void ChangesRos::publishArchitectModelMesh() const {
	cgal_msgs::ProbabilisticMesh p_msg;
	cgal::Polyhedron mesh;
	mesh = mesh_model_.getMesh();
	cgal::triangleMeshToProbMsg(mesh, &p_msg);
	p_msg.header.frame_id = map_frame_;
	p_msg.header.stamp = {secs: 0, nsecs: 0};
	p_msg.header.seq = 0;
	mesh_pub_.publish(p_msg);
}

void ChangesRos::publishColorizedAssocTriangles(const cpt_utils::Associations associations) const {
	// get mesh and convert to redundant ProbabilisticMesh msg
	cgal_msgs::TriangleMesh t_msg;
	cgal_msgs::TriangleMesh t_red_msg;
	cgal_msgs::ProbabilisticMesh p_red_msg; 
	cgal::Polyhedron mesh;
	mesh = mesh_model_.getMesh();
	cgal::triangleMeshToMsg(mesh, &t_msg);
	cgal::createRedundantMsg(t_msg, &t_red_msg);
	cgal::triToProbMsg(t_red_msg, &p_red_msg);

	// set all vertex color to default color
	std::cout << "No of vertices: " << p_red_msg.mesh.vertices.size() << std::endl;

	for (uint i = 0; i < p_red_msg.mesh.vertices.size(); ++i) {
		// can not access vector here with colors[i], because vector size needs to 
		// be defined before assigning values/ has no elements yet
		std_msgs::ColorRGBA c;
		c.r = 0.0;
		c.g = 0.0;
		c.b = 1.0;
		c.a = 0.8;
		p_red_msg.mesh.colors.push_back(c);
	}

	// change color of associated triangles
	for (uint i = 0; i < associations.triangles_to.rows(); ++i) {
		int id = associations.triangles_to(i);

		for (auto vertex : p_red_msg.mesh.triangles[id].vertex_indices) {
			double length = associations.distances(i);
			std_msgs::ColorRGBA c;
			if (discrete_color_ == true) {
				if (length > distance_threshold_) {
					c.r = 1.0;
					c.g = 0.0;
					c.b = 0.0;
					c.a = 0.8;
				} 
				else {
					c.r = 0.0;
					c.g = 1.0;
					c.b = 0.0;
					c.a = 0.8;
				}
			}
			else {
				if (length > 0.4) {
					c.r = 1.0;
					c.g = 0.0;
					c.b = 0.0;
					c.a = 0.8;
				}
				else {
					// create a gradient
					float g = length/0.4; // 1 for red, 0 for green
					if (g > 0.5) {
						c.r = 1.0;
						c.g = 2.0 * (1 - g);
					} else {
						c.r = 2*g;
						c.g = 1.0;
					}
					c.b = 0.0;
					c.a = 0.8;
				}
			}
			p_red_msg.mesh.colors[vertex] = c;
		}
	}

	p_red_msg.header.frame_id = map_frame_;
	p_red_msg.header.stamp = {secs: 0, nsecs: 0};
	p_red_msg.header.seq = 0;
	distance_triangles_pub_.publish(p_red_msg);
}

}
}
