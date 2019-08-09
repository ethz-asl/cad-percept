#ifndef RELATIVE_DEVIATIONS_ROS_H_
#define RELATIVE_DEVIATIONS_ROS_H_

#include <unistd.h>
#include <glog/logging.h>
#include "relative_deviations/deviations.h"
#include "relative_deviations/pc_mesh_creator.h"
#include <cgal_definitions/cgal_typedefs.h>
#include <pcl_ros/point_cloud.h>
#include <cgal_msgs/ColoredMesh.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <cpt_utils/cpt_utils.h>
#include <cpt_utils/pc_processing.h>

#include <boost/circular_buffer.hpp>

#include <map>
#include <unordered_map>

namespace cad_percept {
namespace deviations {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> ColoredPointCloud;

class RelativeDeviations {
  public:
    RelativeDeviations(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    ~RelativeDeviations();

  private:
    Deviations deviations;
    ros::NodeHandle &nh_, nh_private_;
    void publishMesh(const cgal::MeshModel &model, ros::Publisher *publisher) const;
    template <class T>
    void publishCloud(T *cloud, ros::Publisher *publisher) const;
    ros::Publisher ref_mesh_pub_, reading_pc_pub_, icp_pc_pub_, reconstructed_planes_pub_, polygon_pub_, assoc_mesh_pub_, assoc_pc_pub_, assoc_marker_pub_, deviations_mesh_pub_;
    std::string map_frame_;
    bool discrete_color_;
    float score_threshold_;
    /**
     * Publish point cloud of segmented planes
     */
    void publishReconstructedPlanes(const std::vector<reconstructed_plane> &rec_planes, ros::Publisher *publisher) const;
    void publishPolyhedron(cgal::Polyhedron &P);
    void readingCallback(cgal::PointCloud &reading_pc);
    void createTestCase(cgal::PointCloud *reading_pc);
    // create a circular_buffer to store reading pointclouds for alignment
    boost::circular_buffer<cgal::PointCloud> cb; 
    std::string path_;
    void bufferCallback(cgal::PointCloud &reading_pc);

    void publishAssociations(const cgal::MeshModel &model, std::unordered_map<int, polyhedron_plane> &plane_map, const std::vector<reconstructed_plane> &remaining_cloud_vector);

    void publishDeviations(const cgal::MeshModel &model, std::unordered_map<int, polyhedron_plane> &plane_map, std::unordered_map<int, transformation> &transformation_map);

};

}
}

#endif // RELATIVE_DEVIATIONS_ROS_H_