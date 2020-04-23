#include <cgal_conversions/eigen_conversions.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_conversions/tf_conversions.h>
#include <cgal_definitions/mesh_model.h>
#include <cpt_utils/pc_processing.h>
#include <ros/ros.h>

#include "plane_extraction/plane_extraction.h"
#include "test_matcher/map_plane_extractor.h"

void extractPlanesFromCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in);

int main(int argc, char **argv) {
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "           Map Plane Extractor started         " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;

  ros::init(argc, argv, "map_plane_extractor");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string cad_topic_ = nh_private.param<std::string>("cadTopic", "fail");
  ros::Subscriber map_sub = nh.subscribe(cad_topic_, 1, &extractPlanesFromCAD);

  ros::spin();

  return 0;
}

bool file_created = false;
ros::Subscriber map_sub;
ros::Publisher plane_pub;

void extractPlanesFromCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in) {
  if (!file_created) {
    ros::NodeHandle nh_private("~");

    std::string tf_map_frame =
        nh_private.param<std::string>("tfMapFrame", "/map");
    float sample_density = nh_private.param<float>("mapSamplingDensity", 10);
    std::string file_name =
        nh_private.param<std::string>("map_plane_file", "fail");
    std::vector<float> point_in_map = nh_private.param<std::vector<float>>(
        "MapPlaneExtractionPointInMap", {0});

    std::cout << "Processing CAD mesh" << std::endl;
    std::string frame_id = cad_mesh_in.header.frame_id;
    cad_percept::cgal::MeshModel::Ptr reference_mesh;
    cad_percept::cgal::msgToMeshModel(cad_mesh_in.mesh, &reference_mesh);

    // Get transformation from /map to mesh
    tf::StampedTransform transform;
    tf::TransformListener tf_listener(ros::Duration(30));
    try {
      tf_listener.waitForTransform(tf_map_frame, frame_id, ros::Time(0),
                                   ros::Duration(5.0));
      tf_listener.lookupTransform(
          tf_map_frame, frame_id, ros::Time(0),
          transform); // get transformation at latest time T_map_to_frame
    } catch (tf::TransformException ex) {
      ROS_ERROR_STREAM("Couldn't find transformation to mesh system");
    }

    cad_percept::cgal::Transformation ctransformation;
    cad_percept::cgal::tfTransformationToCGALTransformation(transform,
                                                            ctransformation);
    reference_mesh->transform(ctransformation);

    // Sample from mesh
    pcl::PointCloud<pcl::PointXYZ> sample_map;
    int n_points = reference_mesh->getArea() * sample_density;
    cad_percept::cpt_utils::sample_pc_from_mesh(reference_mesh->getMesh(),
                                                n_points, 0.0, &sample_map);

    // Find planes from sampled pc of mesh
    std::vector<Eigen::Vector3d> plane_normals;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> extracted_map_inliers;
    cad_percept::matching_algorithms::PlaneExtractor::cgalRegionGrowing(
        extracted_map_inliers, plane_normals, sample_map, tf_map_frame,
        plane_pub);

    cad_percept::matching_algorithms::MapPlanes map_planes(
        extracted_map_inliers, plane_normals,
        Eigen::Vector3f(point_in_map[0], point_in_map[1], point_in_map[2]));
    map_planes.dispAllPlanes();
    map_planes.saveToYamlFile(file_name);

    std::cout
        << "Saved map file, set create_new_map_file to false to use matcher"
        << std::endl;
  }
}