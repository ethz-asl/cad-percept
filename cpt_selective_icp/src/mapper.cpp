#include "cpt_selective_icp/mapper.h"

namespace cad_percept {
namespace selective_icp {

Mapper::Mapper(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
  : nh_(nh),
    nh_private_(nh_private),
    tf_listener_(ros::Duration(30)),
    odom_received_(0),
    T_scanner_to_map_(PM::TransformationParameters::Identity(4, 4)),
    T_local_map_to_map_(PM::TransformationParameters::Identity(4,4)),
    transformation_(PM::get().REG(Transformation).create("RigidTransformation")) {

  cloud_sub_ = nh_.subscribe(parameters_.scan_topic,
                                     parameters_.input_queue_size,
                                     &Mapper::gotCloud,
                                     this);

  set_ref_srv_ = nh_.advertiseService("set_ref", &Mapper::setReferenceFacets, this);
  reload_icp_config_srv_ = nh_.advertiseService("reload_icp_config", &Mapper::reloadICPConfig, this);
  ref_mesh_pub_ = nh_.advertise<cgal_msgs::ColoredMesh>("ref_mesh", 1, true);
  ref_pc_pub_ = nh_.advertise<PointCloud>("ref_pc", 1, true);
  pose_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("icp_pose", 50, true);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("icp_odom", 50, true);
  scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("corrected_scan", 2, true);

  sleep(5); // wait to set up stuff

  loadICPConfig();
  loadReferenceMap();
}

Mapper::~Mapper() {

}

void Mapper::gotCloud(const sensor_msgs::PointCloud2 &cloud_msg_in) {
  //TODO: why are we doing this if (odom_received_ < 3)?
  if (odom_received_ < 3) {
    try {
      // what do we do with transform? Just a check if it already gets one?
      tf::StampedTransform transform;
      tf_listener_.lookupTransform(parameters_.tf_map_frame,
                                    parameters_.lidar_frame,
                                    cloud_msg_in.header.stamp,
                                    transform);
      odom_received_++;
    } catch (tf::TransformException ex) {
      ROS_WARN_STREAM("Transformations still initializing.");
      // why do we even publish this identity matrix?
      pose_pub_.publish(PointMatcher_ros::eigenMatrixToTransformStamped
                            <float>(
          T_scanner_to_map_.inverse(),
          parameters_.lidar_frame,
          parameters_.tf_map_frame,
          cloud_msg_in.header.stamp));
      odom_received_++;
    }
  } else {
    DP cloud(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloud_msg_in));

    processCloud(cloud,
                  cloud_msg_in.header.stamp,
                  cloud_msg_in.header.seq);
  }
}

void Mapper::processCloud(DP &point_cloud,
                          const ros::Time &stamp,
                          uint32_t seq) {

  if (parameters_.sensor_frame == "") {
    parameters_.sensor_frame = parameters_.lidar_frame;
  }

  PointMatcherSupport::timer t;

  const size_t good_count(point_cloud.features.cols());
  if (good_count == 0) {
    ROS_ERROR("[ICP] I found no good points in the cloud");
    return;
  }

  // Dimension of the point cloud, important since we handle 2D and 3D.
  const int dimp1(point_cloud.features.rows());

  // This need to be depreciated, there is addTime for those field in pm.
  if (!(point_cloud.descriptorExists("stamps_Msec")
      && point_cloud.descriptorExists("stamps_sec")
      && point_cloud.descriptorExists("stamps_nsec"))) {
    const float Msec = round(stamp.sec / 1e6);
    const float sec = round(stamp.sec - Msec * 1e6);
    const float nsec = round(stamp.nsec);

    const PM::Matrix desc_Msec = PM::Matrix::Constant(1, good_count, Msec);
    const PM::Matrix desc_sec = PM::Matrix::Constant(1, good_count, sec);
    const PM::Matrix desc_nsec = PM::Matrix::Constant(1, good_count, nsec);
    point_cloud.addDescriptor("stamps_Msec", desc_Msec);
    point_cloud.addDescriptor("stamps_sec", desc_sec);
    point_cloud.addDescriptor("stamps_nsec", desc_nsec);
  }

  int pts_count = point_cloud.getNbPoints();
  if (pts_count < parameters_.min_reading_point_count) {
    ROS_ERROR_STREAM(
        "[ICP] Not enough points in new PointCloud: only " << pts_count
                                                          << " pts.");
    return;
  }

  //TODO: remove if not needed (do it in icp config anyway)
  input_filters_.apply(point_cloud);

  try {
    T_scanner_to_map_ = PointMatcher_ros::eigenMatrixToDim<float>(
        PointMatcher_ros::transformListenerToEigenMatrix<float>(
            tf_listener_,
            parameters_.tf_map_frame, // to
            parameters_.lidar_frame, // from
            stamp
        ), dimp1);
  } catch (tf::ExtrapolationException e) {
    ROS_ERROR_STREAM("Extrapolation Exception. stamp = " << stamp << " now = "
                                                         << ros::Time::now()
                                                         << " delta = "
                                                         << ros::Time::now()
                                                             - stamp << std::endl
                                                         << e.what());
    return;
  } catch (...) {
    // Everything else.
    ROS_ERROR_STREAM("Unexpected exception... ignoring scan.");
    return;
  }
  ROS_DEBUG_STREAM(
      "[ICP] T_scanner_to_map (" << parameters_.lidar_frame << " to "
                                 << parameters_.tf_map_frame << "):\n"
                                 << T_scanner_to_map_);

  // correctParameters: force orthogonality of rotation matrix
  // what is meaning of local_map?
  const PM::TransformationParameters T_scanner_to_local_map =
      transformation_->correctParameters(
          T_local_map_to_map_.inverse() * T_scanner_to_map_);

  pts_count = point_cloud.getNbPoints(); // because filter between
  if (pts_count < parameters_.min_reading_point_count) {
    ROS_ERROR_STREAM(
        "[ICP] Not enough points in newPointCloud: only " << pts_count
                                                          << " pts.");
    return;
  }

  if (point_cloud.getEuclideanDim()
      != ref_dp.getEuclideanDim()) {
    ROS_ERROR_STREAM("[ICP] Dimensionality missmatch: incoming cloud is "
                         << point_cloud.getEuclideanDim()
                         << " while reference is "
                         << ref_dp.getEuclideanDim());
    return;
  }

  try {
    PM::TransformationParameters T_updated_scanner_to_map;
    PM::TransformationParameters T_updated_scanner_to_local_map;

    std::cout << "DEBUG 1" << std::endl;

    ROS_DEBUG_STREAM(
        "[ICP] Computing - reading: " << point_cloud.getNbPoints()
                                      << ", reference: "
                                      << ref_dp.getNbPoints());

    // Do ICP
    std::cout << "DEBUG 2" << std::endl;
    T_updated_scanner_to_local_map = icp_(point_cloud, ref_dp,
                                          T_scanner_to_local_map);

    std::cout << "DEBUG 2.1" << std::endl;
    T_updated_scanner_to_map = T_local_map_to_map_ *
        T_updated_scanner_to_local_map;

    ROS_DEBUG_STREAM(
        "[ICP] T_updatedScanner_to_map:\n" << T_updated_scanner_to_map);
    ROS_DEBUG_STREAM("[ICP] T_updatedScanner_to_localMap:\n"
                         << T_updated_scanner_to_local_map);

    // Ensure minimum overlap between scans.
    std::cout << "DEBUG 3" << std::endl;
    const double estimated_overlap = icp_.errorMinimizer->getOverlap();
    ROS_DEBUG_STREAM("[ICP] Overlap: " << estimated_overlap);
    if (estimated_overlap < parameters_.min_overlap) {
      ROS_ERROR_STREAM(
          "[ICP] Estimated overlap too small, ignoring ICP correction!");
      return;
    }

    // Publish odometry.
    std::cout << "DEBUG 4" << std::endl;
    if (odom_pub_.getNumSubscribers()) {
      odom_pub_.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(
          T_updated_scanner_to_map,
          parameters_.tf_map_frame,
          stamp));
    }
    // Publish pose. Same as odometry, but different msg type.
    if (pose_pub_.getNumSubscribers()) {
      pose_pub_.publish(PointMatcher_ros::eigenMatrixToTransformStamped<float>(
          T_updated_scanner_to_map,
          parameters_.lidar_frame,
          parameters_.tf_map_frame,
          stamp));
    }

    // Publish the corrected scan point cloud
    std::cout << "DEBUG 5" << std::endl;
    DP pc = transformation_->compute(point_cloud,
                                     T_updated_scanner_to_map);
    map_post_filters_.apply(pc); // not there atm

    if (scan_pub_.getNumSubscribers()) {
      ROS_DEBUG_STREAM(
          "Corrected scan publishing " << pc.getNbPoints() << " points");
      scan_pub_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(pc,
                                                                           parameters_.tf_map_frame,
                                                                           stamp));
    }
    std::cout << "DEBUG 6" << std::endl;

  } catch (PM::ConvergenceError error) {
    ROS_ERROR_STREAM("[ICP] failed to converge: " << error.what());
    return;
  } catch(const std::exception &ex) {
    std::cerr << "Error occured: " << ex.what() << std::endl;
  } catch (...) {
    // everything else.
    ROS_ERROR_STREAM("Unen XZ");
    return;
  }
  // Statistics about time and real-time capability.
  int real_time_ratio =
      100 * t.elapsed() / (stamp.toSec() - last_poin_cloud_time_.toSec());
  real_time_ratio *= seq - last_point_cloud_seq_;

  ROS_INFO_STREAM("[TIME] Total ICP took: " << t.elapsed() << " [s]");
  if (real_time_ratio < 80)
    ROS_INFO_STREAM("[TIME] Real-time capability: " << real_time_ratio <<
                                                    "%");
  else
    ROS_WARN_STREAM("[TIME] Real-time capability: " << real_time_ratio << "%");

  last_poin_cloud_time_ = stamp;
  last_point_cloud_seq_ = seq;
}

void Mapper::loadReferenceMap() {
  reference_mesh_.init(parameters_.reference_mesh.c_str());
  cgal::Polyhedron P = reference_mesh_.getMesh();

  // Make some checks:
  if (P.is_valid()) {
    std::cout << "P is valid" << std::endl;
  }
  else {
    std::cerr << "P is not valid" << std::endl;
  }
  if (P.is_pure_triangle()) {
    std::cout << "P is pure triangle" << std::endl;
  }
  else {
    std::cerr << "P is not pure triangle" << std::endl;
  }
  if (P.is_closed()) {
    std::cout << "P is closed" << std::endl;
  }
  else {
    std::cerr << "P is not closed" << std::endl;
  }


  // first extract whole ref_pointcloud, ref_pointcloud will be changed later according to references
  std::unordered_set<int> references; // empty
  extractReferenceFacets(parameters_.map_sampling_density, reference_mesh_, references, &ref_pointcloud);
  ref_dp = deviations::pointCloudToDP(ref_pointcloud);
}

bool Mapper::setReferenceFacets(cpt_selective_icp::References::Request &req,
                                cpt_selective_icp::References::Response &res) {
  std::unordered_set<int> references;
  for (auto id : req.data) {
    references.insert(id);
  }
  extractReferenceFacets(parameters_.map_sampling_density, reference_mesh_, references, &ref_pointcloud);
  return true;
}

void Mapper::publishReferenceMesh(cgal::MeshModel &reference_mesh, std::unordered_set<int> &references) {
  cgal::Polyhedron P;
  P = reference_mesh.getMesh();
  cgal_msgs::TriangleMesh t_msg;
  cgal_msgs::ColoredMesh c_msg;
  cgal::triangleMeshToMsg(P, &t_msg);
  c_msg.mesh = t_msg;
  c_msg.header.frame_id = parameters_.tf_map_frame;
  c_msg.header.stamp = {secs: 0, nsecs: 0};
  c_msg.header.seq = 0;

  std_msgs::ColorRGBA c;
  
  // set al facet colors to blue
  for (uint i = 0; i < c_msg.mesh.triangles.size(); ++i) {
    c.r = 0.0;
    c.g = 0.0;
    c.b = 1.0;
    c.a = 0.4;
    c_msg.colors.push_back(c);  
  }

  // set reference facets to red
  for (auto reference : references) {
    c.r = 1.0;
    c.g = 0.0;
    c.b = 0.0;
    c.a = 0.4;
    c_msg.colors[reference] = c;
  }
  ref_mesh_pub_.publish(c_msg);
}

template <class T>
void Mapper::publishCloud(T *cloud, ros::Publisher *publisher) const {
  cloud->header.frame_id = parameters_.tf_map_frame;
  pcl_conversions::toPCL(ros::Time(0), cloud->header.stamp);
  publisher->publish(*cloud);
}

void Mapper::extractReferenceFacets(const int density, cgal::MeshModel &reference_mesh, std::unordered_set<int> &references, PointCloud *pointcloud) {
  pointcloud->clear();
  // if reference set is empty, extract whole point cloud
  if (references.empty() == 1) {
    std::cout << "Extract whole point cloud (normal ICP)" << std::endl;
    int n_points = reference_mesh.getArea() * density;
    cgal::sample_pc_from_mesh(reference_mesh.getMesh(), n_points, 0.0, pointcloud, "ref_pointcloud");
    publishReferenceMesh(reference_mesh, references);
  }
  else {
    std::cout << "Extract selected reference point clouds (selective ICP)" << std::endl;

    // sample reference point cloud from mesh

    // get coplanar facets
    std::unordered_set<int> references_new;

    std::cout << "Choosen reference facets:" << std::endl;
    std::unordered_set<int>::iterator uitr;
    for (uitr = references.begin(); uitr != references.end(); uitr++) {
      std::cout << *uitr << std::endl;
      reference_mesh.findCoplanarFacets(*uitr, &references_new);
    }

    std::cout << "Computed reference facets:" << std::endl;
    for (uitr = references_new.begin(); uitr != references_new.end(); uitr++) {
      std::cout << *uitr << std::endl;
    }

    publishReferenceMesh(reference_mesh, references_new);

    // create sampled point cloud from reference_new
    
    // generated points
    std::vector<cgal::Point> points;
    // create input triangles
    std::vector<cgal::Triangle> triangles;
    double reference_area = 0;
    cgal::Polyhedron P = reference_mesh.getMesh();
    for (cgal::Polyhedron::Facet_iterator j = P.facets_begin(); j != P.facets_end(); ++j) {
      if (references_new.find(j->id()) != references_new.end()) {
        cgal::Triangle t(j->halfedge()->vertex()->point(),
                         j->halfedge()->next()->vertex()->point(),
                         j->halfedge()->next()->next()->vertex()->point());
        reference_area += CGAL::to_double(sqrt(t.squared_area()));
        triangles.push_back(t);
      }
    }

    // Create the generator, input is the vector of Triangle
    CGAL::Random_points_in_triangles_3<cgal::Point> g(triangles);
    // Get no_of_points random points in cdt
    int no_of_points = reference_area * density;
    CGAL::cpp11::copy_n(g, no_of_points, std::back_inserter(points));
    // Check that we have really created no_of_points.
    assert(points.size() == no_of_points);

    for (auto point : points) {
      pcl::PointXYZ cloudpoint;
      cloudpoint.x = (float)point.x();
      cloudpoint.y = (float)point.y();
      cloudpoint.z = (float)point.z();
      pointcloud->push_back(cloudpoint);
    }
  }
  publishCloud<PointCloud>(pointcloud, &ref_pc_pub_);
}

void Mapper::loadICPConfig() {
  // Load configs.
  std::string config_file_name;
  if (ros::param::get("~icpConfig", config_file_name)) {
    std::ifstream ifs(config_file_name.c_str());
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
    std::ifstream ifs(config_file_name.c_str());
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
    std::ifstream ifs(config_file_name.c_str());
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
    std::ifstream ifs(config_file_name.c_str());
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
