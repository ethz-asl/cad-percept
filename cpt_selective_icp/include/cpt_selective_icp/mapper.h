#ifndef SELECTIVE_ICP_MAPPER_H_
#define SELECTIVE_ICP_MAPPER_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "cpt_selective_icp/mapper_parameters.h"
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <pcl/point_types.h>
#include "relative_deviations/pc_mesh_creator.h"

#include "cpt_selective_icp/References.h"


namespace cad_percept {
namespace mapper {

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
    MapperParameters parameters_;
    cgal::MeshModel reference_mesh_;
    bool setReferenceFacets(cad_percept::mapper::References &req,
                            cad_percept::mapper::Response &res);
    PointCloud ref_pointcloud;



    int odom_received_;
    tf::TransformListener tf_listener_;

    PM::TransformationParameters T_scanner_to_map_;

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

    // Services
    ros::ServiceServer set_ref_srv_;


};

}
}

#endif // SELECTIVE_ICP_MAPPER_H_