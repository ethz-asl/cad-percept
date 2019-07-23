#ifndef SELECTIVE_ICP_MAPPER_H_
#define SELECTIVE_ICP_MAPPER_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "cpt_selective_icp/mapper_parameters.h"
#include "cgal_definitions/cgal_typedefs.h"
#include "cgal_definitions/mesh_model.h"
#include <pcl/point_types.h>
#include "relative_deviations/pc_mesh_creator.h"
#include "pointmatcher/PointMatcher.h"
#include <unordered_set>
#include "cgal_conversions/mesh_conversions.h"
#include <boost/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp>
#include <boost/bimap/unordered_multiset_of.hpp>

#include "cpt_selective_icp/References.h"
#include <std_srvs/Empty.h>
#include <cgal_msgs/TriangleMesh.h>
#include <cgal_msgs/ColoredMesh.h>
#include <std_msgs/ColorRGBA.h>


namespace cad_percept {
namespace selective_icp {

typedef boost::bimap<boost::bimaps::unordered_set_of<int>, boost::bimaps::unordered_multiset_of<int>> association_bimap;
typedef association_bimap::value_type bi_association;

typedef PointMatcher<float> PM;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef std::map<int,int>::iterator Miterator;
typedef std::multimap<int,int>::iterator Mmiterator;


class Mapper {
  public:
    Mapper(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    ~Mapper();

  private:
    ros::NodeHandle &nh_, &nh_private_;
    tf::TransformListener tf_listener_;
    MapperParameters parameters_;
    cgal::MeshModel reference_mesh_;
    void gotCloud(const sensor_msgs::PointCloud2 &cloud_msg_in);
    bool setReferenceFacets(cpt_selective_icp::References::Request &req,
                            cpt_selective_icp::References::Response &res);
    void publishReferenceMesh(cgal::MeshModel &reference_mesh, std::unordered_set<int> &references);
    template <class T>
    void publishCloud(T *cloud, ros::Publisher *publisher) const;
    PointCloud ref_pointcloud;
    void extractReferenceFacets(const int density, cgal::MeshModel &reference_mesh, std::unordered_set<int> &references, PointCloud *pointcloud);

    void loadReferenceMap();

    /**
     * Load parameters for libpointmatcher from yaml
     */
    void loadICPConfig();
    bool reloadICPConfig(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res);

    // Subscribers
    ros::Subscriber cloud_sub_;

    // Publishers
    ros::Publisher ref_mesh_pub_;
    ros::Publisher ref_pc_pub_;

    // Services
    ros::ServiceServer set_ref_srv_;
    ros::ServiceServer reload_icp_config_srv_;

    // libpointmatcher
    PM::ICP icp_;
    PM::DataPointsFilters input_filters_;
    PM::DataPointsFilters map_pre_filters_;
    PM::DataPointsFilters map_post_filters_;





};

}
}

#endif // SELECTIVE_ICP_MAPPER_H_
