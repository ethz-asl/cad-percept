#include "cpt_selective_icp/mapper.h"

namespace cad_percept {
namespace mapper {

Mapper::Mapper(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
  : nh_(nh),
    nh_private_(nh_private),
    tf_listener_(ros::Duration(30)),
    odom_received_(0) {

  loadICPConfig();
  loadReferenceMap();

  cloud_sub_ = nh_private_.subscribe(parameters_.scan_topic,
                                     parameters_.input_queue_size,
                                     &Mapper::gotCloud,
                                     this);

  set_ref_srv_ = nh_private_.advertiseService("set_ref", &Mapper::setReferenceFacets, this);
}

Mapper::~Mapper() {

}

void Mapper::gotCloud(const sensor_msgs::PointCloud2 &cloud_msg_in) {
  if (odom_received_ < 3) { // why?
    try {
      tf::StampedTransform transform;
      tf_listener_.lookupTransform(parameters_.tf_map_frame, // destination
                                  parameters_.lidar_frame,  // original
                                  cloud_msg_in.header.stamp, // take transform corresponding to p.c. time
                                  transform);
      odom_received_++;
    } catch (tf::TransformException ex) {
      ROS_WARN_STREAM("Transformations still initializing.");
      pose_pub_.publish(PointMatcher_ros::eigenMatrixToTransformStamped // why do we publish identity matrix?
                            <float>(
          T_scanner_to_map_.inverse(), // transform to save
          parameters_.lidar_frame, // target, child frame
          parameters_.tf_map_frame, // source
          cloud_msg_in.header.stamp));
      odom_received_++;
    }
  } else {
    unique_ptr<DP> cloud
        (new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(
            cloud_msg_in)));

    processCloud(move(cloud),
                parameters_.lidar_frame,
                cloud_msg_in.header.stamp,
                cloud_msg_in.header.seq, false);
  }
}

void Mapper::loadReferenceMap() {
  reference_mesh_.init(parameters_.reference_mesh.c_str());
  // first extract whole ref_pointcloud, ref_pointcloud will be changed later according to references
  extractReferenceFacets(parameters_.map_sampling_density, reference_mesh_, std::unordered_set<int> references, &ref_pointcloud) {
}

bool Mapper::setReferenceFacets(cad_percept::mapper::References &req,
                                cad_percept::mapper::Response &res) {
  std::unordered_set<int> references;
  for (auto id : req.data) {
    references.insert(id);
  }
  extractReferenceFacets(parameters_.map_sampling_density, reference_mesh_, references, &ref_pointcloud);
  return true;
}

void Mapper::extractReferenceFacets(const int density, cgal::MeshModel &reference_mesh, std::unordered_set<int> &references, PointCloud *pointcloud) {
  // if reference set is empty, extract whole point cloud
  if (references.empty() == 1) {
    std::cout << "Extract whole point cloud (normal ICP)" << std::endl;
    int n_points = reference_mesh.getArea() * density;
    cgal::sample_pc_from_mesh(reference_mesh.getMesh(), n_points, 0.0, pointcloud, "ref_pointcloud");
  }
  else {
    std::cout << "Extract selected reference point clouds (selective ICP)" << std::endl;
    
    std::cout << "Choosen reference facets:" << std::endl;
    std::unordered_set<int>::iterator uitr;
    for (uitr = references.begin(); uitr != references.end(); uitr++) {
      std::cout << *uitr << std::endl;
    }

    // sample reference point cloud from mesh

    // use mergeCoplanarFacets to get coplanar facets
    cgal::Polyhedron P_merged; // not used
    std::multimap<int, int> merge_associations; // new ID to old ID
    std::map<int, int> merge_associations_inv; // old ID to new ID
    reference_mesh.mergeCoplanarFacets(&P_merged, &merge_associations);

    // create the inverse map
    for (Mmiterator it = merge_associations.begin(); it != merge_associations.end(); it = merge_associations.upper_bound(it->first)) {
      auto iit = merge_associations_old.equal_range(it->first);
      for (auto itr = iit.first; itr != iit.second; ++itr) {
        merge_associations_inv.insert(std::make_pair(itr->second, itr->first));
      }
    }

    // create unordered set of all neighboring colinear facets
    std::unordered_set<int> references_new;
    for (auto reference : references) {
      Miterator it = merge_associations_inv.find(reference);
      auto iit = merge_associations.equal_range(it->second);
      for (auto itr = iit.first; itr != iit.second; ++itr) {
        if (references_new.find(itr->second) == references_new.end()) {
          references_new.insert(itr->second);
        }
      }
    }

    // continue here by getting point clouds of references with density



  }
}


void Mapper::loadICPConfig() {
  // Load configs.
  string config_file_name;
  if (ros::param::get("~icpConfig", config_file_name)) {
    ifstream ifs(config_file_name.c_str());
    if (ifs.good()) {
      icp_.loadFromYaml(ifs);
    } else {
      ROS_ERROR_STREAM(
          "Cannot load ICP config from YAML file " << config_file_name);
      icp_.setDefault();
    }
  } else {
    ROS_INFO_STREAM("No ICP config file given, using default");
    icp_.setDefault();
  }

  if (ros::param::get("~inputFiltersConfig", config_file_name)) {
    ifstream ifs(config_file_name.c_str());
    if (ifs.good()) {
      input_filters_ = PM::DataPointsFilters(ifs);
    } else {
      ROS_ERROR_STREAM(
          "Cannot load input filters config from YAML file "
              << config_file_name);
    }
  } else {
    ROS_INFO_STREAM(
        "No input filters config file given, not using these filters");
  }

  if (ros::param::get("~mapPreFiltersConfig", config_file_name)) {
    ifstream ifs(config_file_name.c_str());
    if (ifs.good()) {
      map_pre_filters_ = PM::DataPointsFilters(ifs);
    } else {
      ROS_ERROR_STREAM("Cannot load map pre-filters config from YAML file "
                           << config_file_name);
    }
  } else {
    ROS_INFO_STREAM(
        "No map pre-filters config file given, not using these filters");
  }

  if (ros::param::get("~mapPostFiltersConfig", config_file_name)) {
    ifstream ifs(config_file_name.c_str());
    if (ifs.good()) {
      map_post_filters_ = PM::DataPointsFilters(ifs);
    } else {
      ROS_ERROR_STREAM("Cannot load map post-filters config from YAML file "
                           << config_file_name);
    }
  } else {
    ROS_INFO_STREAM(
        "No map post-filters config file given, not using these filters");
  }
}

bool Mapper::reloadICPConfig(std_srvs::Empty::Request &req,
                           std_srvs::Empty::Response &res) {
  loadICPConfig();
  ROS_INFO_STREAM("Parameters reloaded");

  return true;
}

}
}