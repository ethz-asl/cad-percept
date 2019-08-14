#include "cpt_selective_icp/mapper.h"

namespace cad_percept {
namespace selective_icp {

Mapper::Mapper(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
  : nh_(nh),
    nh_private_(nh_private),
    tf_listener_(ros::Duration(30)),
    odom_received_(0),
    T_scanner_to_map_(PM::TransformationParameters::Identity(4, 4)),
    transformation_(PM::get().REG(Transformation).create("RigidTransformation")),
    cad_trigger(false),
    selective_icp_trigger(false),
    normal_icp_trigger(true),
    ref_mesh_ready(false),
    projection_count(0) {

  cloud_sub_ = nh_.subscribe(parameters_.scan_topic,
                             parameters_.input_queue_size,
                             &Mapper::gotCloud,
                             this);
  cad_sub_ = nh_.subscribe(parameters_.cad_topic,
                           parameters_.input_queue_size,
                           &Mapper::gotCAD,
                           this);

  load_published_map_srv_ = nh_private_.advertiseService("load_published_map", &Mapper::loadPublishedMap, this);
  get_closest_facet_srv_ = nh_private_.advertiseService("get_closest_facet", &Mapper::getClosestFacet, this);
  set_ref_srv_ = nh_.advertiseService("set_ref", &Mapper::setReferenceFacets, this);
  set_normal_icp_srv_ = nh_.advertiseService("normal_icp", &Mapper::setNormalICP, this);
  set_selective_icp_srv_ = nh_.advertiseService("selective_icp", &Mapper::setSelectiveICP, this);
  reload_icp_config_srv_ = nh_.advertiseService("reload_icp_config", &Mapper::reloadConfig, this);

  ref_mesh_pub_ = nh_.advertise<cgal_msgs::ColoredMesh>("ref_mesh", 1, true);
  cad_mesh_pub_ = nh_.advertise<cgal_msgs::TriangleMeshStamped>("cad_mesh_pub", 1, true);
  ref_pc_pub_ = nh_.advertise<PointCloud>("ref_pc", 1, true);
  pose_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("icp_pose", 50, true);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("icp_odom", 50, true);
  scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("corrected_scan", 2, true);
  selective_icp_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("ref_corrected_scan", 2, true);
  point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("point_pub_", 2, true);
  map_pub_ = nh_.advertise<PointCloud>("map", 1, true);

  std::cout << "Wait for start-up" << std::endl;
  sleep(5); // wait to set up stuff
  std::cout << "Ready!" << std::endl;

  loadConfig();
}

Mapper::~Mapper() {

}

bool Mapper::loadPublishedMap(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &res) {
// Since CAD is published all the time, we need a trigger when to load it
  cad_trigger = true;
  return true;
}

bool Mapper::getClosestFacet(cpt_selective_icp::FacetID::Request &req,
                             cpt_selective_icp::FacetID::Response &res) {
  if (ref_mesh_ready == false) {
    std::cerr << "Reference mesh not loaded yet or not ready!" << std::endl;
  }
  else {
    cgal::PointAndPrimitiveId ppid = 
      reference_mesh_.getClosestPrimitive((double)req.point.x, (double)req.point.y, (double)req.point.z);
    cgal::Point pt = ppid.first;
    int facet_id = reference_mesh_.getFacetIndex(ppid.second);
    res.facet_id = facet_id;

    // project this point on corresponding facet
    geometry_msgs::Point msg;
    geometry_msgs::PointStamped smsg;
    cgal::pointToMsg(pt, &msg);
    smsg.point = msg;
    smsg.header.seq = projection_count;
    smsg.header.frame_id = parameters_.tf_map_frame;
    smsg.header.stamp = ros::Time(0);
    point_pub_.publish(smsg);
    projection_count++;
  }
  return true;
}

void Mapper::gotCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in) {
  if (cad_trigger) {
    std::cout << "Processing CAD mesh" << std::endl;
    std::string frame_id = cad_mesh_in.header.frame_id; // should be "marker2" 
    cgal::Polyhedron P;
    cgal::msgToTriangleMesh(cad_mesh_in.mesh, &P);

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
      std::cerr << "P is not closed => no consistent normal directions" << std::endl;
    }

    reference_mesh_.init(P);

    tf::StampedTransform transform;
    tf_listener_.lookupTransform(parameters_.tf_map_frame, frame_id, ros::Time(0), transform); // from tf_map_frame to "marker2"
    Eigen::Matrix3d rotation;
    tf::matrixTFToEigen(transform.getBasis(), rotation);
    Eigen::Vector3d translation;
    tf::vectorTFToEigen(transform.getOrigin(), translation);
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block(0, 0, 3, 3) = rotation;
    transformation.block(0, 3, 3, 1) = translation;
    cgal::Transformation ctransformation;
    cgal::eigenTransformationToCgalTransformation(transformation, &ctransformation);
    reference_mesh_.transform(ctransformation);
    ref_mesh_ready = true;

    //TODO: extract reference facets of all facets ==> get complete point cloud, then do p.c. pre-processing, then set as map in ICPSeq

    // first extract whole pointcloud
    std::unordered_set<int> references; // empty
    PointCloud pointcloud;
    extractReferenceFacets(parameters_.map_sampling_density, reference_mesh_, references, &pointcloud);

    ref_dp = cpt_utils::pointCloudToDP(pointcloud);
    processCloud(&ref_dp, ros::Time(0));

    // set the map
    icp_.clearMap();
    icp_.setMap(ref_dp);

    // publish transformed reference mesh once for relative_deviations (avoid reloading)
    publishMesh(reference_mesh_, &cad_mesh_pub_);

    cad_trigger = false;
  }
}

void Mapper::publishMesh(const cgal::MeshModel &model, ros::Publisher *publisher) const {
  cgal::Polyhedron P;
  P = model.getMesh();
  cgal_msgs::TriangleMesh t_msg;
  cgal_msgs::TriangleMeshStamped s_msg;
  cgal::triangleMeshToMsg(P, &t_msg);
  s_msg.mesh = t_msg;

  s_msg.header.frame_id = parameters_.tf_map_frame;
  s_msg.header.stamp = {secs: 0, nsecs: 0};
  publisher->publish(s_msg);
}

void Mapper::gotCloud(const sensor_msgs::PointCloud2 &cloud_msg_in) {
  /**
   * Wait until tf_listener gets results
   */
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
    boost::thread map_thread;
    ros::Time stamp = cloud_msg_in.header.stamp;
    uint32_t seq = cloud_msg_in.header.seq;

    /**
     * Check if initial map was loaded and we are ready to go
     */
    if(!icp_.hasMap()) {
      std::cout << "ICP not initialized yet" << std::endl;
      
      // Publish identity matrix as long as no map to avoid drift from IMU during alignment
      // Publish odometry.
      if (odom_pub_.getNumSubscribers()) {
        odom_pub_.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(
            T_scanner_to_map_,
            parameters_.tf_map_frame,
            stamp));
      }
      // Publish pose. Same as odometry, but different msg type.
      if (pose_pub_.getNumSubscribers()) {
        pose_pub_.publish(PointMatcher_ros::eigenMatrixToTransformStamped<float>(
            T_scanner_to_map_,
            parameters_.lidar_frame,
            parameters_.tf_map_frame,
            stamp));
      }
      return; // cancel if icp_ was not initialized yet
    }

    DP cloud(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloud_msg_in));
    
    PointMatcherSupport::timer t;
    processCloud(&cloud,
                  cloud_msg_in.header.stamp);

    // Cloud specific ICP preparation:

    // Dimension of the point cloud, important since we handle 2D and 3D.
    const int dimp1(cloud.features.rows());
    // Ensure a minimum amount of points, otherwise cancel ICP
    int pts_count = cloud.getNbPoints();
    if (pts_count < parameters_.min_reading_point_count) {
      ROS_ERROR_STREAM(
          "[ICP] Not enough points in reading pointcloud: only " << pts_count
                                                            << " pts.");
      return;
    }

    /**
     * Get transform
     */
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
    T_scanner_to_map_ = transformation_->correctParameters(T_scanner_to_map_);

    // Check dimension
    if (cloud.getEuclideanDim()
        != icp_.getPrefilteredInternalMap().getEuclideanDim()) { // icp_ and selective_icp_ give same here
      ROS_ERROR_STREAM("[ICP] Dimensionality missmatch: incoming cloud is "
                          << cloud.getEuclideanDim()
                          << " while map is "
                          << icp_.getPrefilteredInternalMap().getEuclideanDim());
      return;
    }

    PM::TransformationParameters T_updated_scanner_to_map = T_scanner_to_map_;
    DP dp;

    /**
     * Normal ICP
     */
    if (normal_icp_trigger == true) {
      std::cout << "Perform normal ICP" << std::endl;
      try {

        ROS_DEBUG_STREAM(
            "[ICP] Computing - reading: " << cloud.getNbPoints()
                                          << ", reference: "
                                          << icp_.getInternalMap().getNbPoints());

        // Do ICP
        T_updated_scanner_to_map = icp_(cloud, T_scanner_to_map_);

        ROS_DEBUG_STREAM(
            "[ICP] T_updated_scanner_to_map_normal:\n" << T_updated_scanner_to_map);

        // Ensure minimum overlap between scans.
        const double estimated_overlap = icp_.errorMinimizer->getOverlap();
        ROS_DEBUG_STREAM("[ICP] Overlap: " << estimated_overlap);
        if (estimated_overlap < parameters_.min_overlap) {
          ROS_ERROR_STREAM(
              "[ICP] Estimated overlap too small, ignoring ICP correction!");
          return;
        }
        dp = ref_dp;

      } catch (PM::ConvergenceError error) {
        ROS_ERROR_STREAM("[ICP] failed to converge: " << error.what());
        return;
      } catch(const std::exception &ex) {
        std::cerr << "Error occured: " << ex.what() << std::endl;
        return;
      } catch (...) {
        // everything else.
        ROS_ERROR_STREAM("Unen XZ");
        return;
      }
    }

    /**
     * Selective ICP
     */
    if (selective_icp_trigger == true) {
      std::cout << "Perform selective ICP" << std::endl;
      try {
        ROS_DEBUG_STREAM(
            "[Selective ICP] Computing - reading: " << cloud.getNbPoints()
                                          << ", reference: "
                                          << selective_icp_.getInternalMap().getNbPoints());

        // Do ICP
        T_updated_scanner_to_map = selective_icp_(cloud, T_updated_scanner_to_map);

        ROS_DEBUG_STREAM(
            "[Selective ICP] T_updated_scanner_to_map_selective:\n" << T_updated_scanner_to_map);

        // Ensure minimum overlap between scans.
        const double estimated_overlap = selective_icp_.errorMinimizer->getOverlap();
        ROS_DEBUG_STREAM("[Selective ICP] Overlap: " << estimated_overlap);
        if (estimated_overlap < parameters_.min_overlap) {
          ROS_ERROR_STREAM(
              "[Selective ICP] Estimated overlap too small, ignoring ICP correction!");
          return;
        }


        // Publish the selective ICP exclusive transformed cloud for deviation analysis
        DP pc = transformation_->compute(cloud,
                                        T_updated_scanner_to_map);

        if (parameters_.mapping_trigger == true) {
          map_thread = boost::thread(&Mapper::addScanToMap, this, pc, stamp);
        }

        if (selective_icp_scan_pub_.getNumSubscribers()) {
          ROS_DEBUG_STREAM(
              "Selective ICP scan publishing " << pc.getNbPoints() << " points");
          selective_icp_scan_pub_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(pc,
                                                                              parameters_.tf_map_frame,
                                                                              stamp));
        }
        dp = selective_ref_dp;

      } catch (PM::ConvergenceError error) {
        ROS_ERROR_STREAM("[Selective ICP] failed to converge: " << error.what());
        if (normal_icp_trigger == false) {
          return;
        }
        ROS_ERROR_STREAM("[Selective ICP] Using result from normal ICP");
      } catch(const std::exception &ex) {
        std::cerr << "Error occured: " << ex.what() << std::endl;
        if (normal_icp_trigger == false) {
          return;
        }
        ROS_ERROR_STREAM("[Selective ICP] Using result from normal ICP");
      } catch (...) {
        // everything else.
        ROS_ERROR_STREAM("Unen XZ");
        if (normal_icp_trigger == false) {
          return;
        }
        ROS_ERROR_STREAM("[Selective ICP] Using result from normal ICP");
      }
    }

    if (normal_icp_trigger == false && selective_icp_trigger == false) {
      std::cerr << "Error in code. Both ICP methods turned off." << std::endl;
      return;
    }

    /**
     * Publish
     */ 

    // Publish odometry.
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
    DP pc = transformation_->compute(cloud,
                                    T_updated_scanner_to_map);

    if (scan_pub_.getNumSubscribers()) {
      ROS_DEBUG_STREAM(
          "Corrected scan publishing " << pc.getNbPoints() << " points");
      scan_pub_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(pc,
                                                                          parameters_.tf_map_frame,
                                                                          stamp));
    }

    /**
     *  ICP error metrics
     */
    getICPError(pc);
    if (!references_new.empty()) {
      getICPErrorToRef(pc);
    }
    
    getError(dp, pc);

    map_thread.join();
    
    /** 
     * Statistics about time and real-time capability.
     */
    int real_time_ratio =
        100 * t.elapsed() / (stamp.toSec() - last_point_cloud_time_.toSec());
    real_time_ratio *= seq - last_point_cloud_seq_;

    ROS_INFO_STREAM("[TIME] Total ICP took: " << t.elapsed() << " [s]");
    if (real_time_ratio < 80)
      ROS_INFO_STREAM("[TIME] Real-time capability: " << real_time_ratio <<
                                                      "%");
    else
      ROS_WARN_STREAM("[TIME] Real-time capability: " << real_time_ratio << "%");

    last_point_cloud_time_ = stamp;
    last_point_cloud_seq_ = seq;               
  }
}

double Mapper::getICPErrorToRef(const DP &aligned_dp) {
  PointCloud aligned_pc = cpt_utils::dpToPointCloud(aligned_dp);
  int point_count = 0;
  double result = 0;
  for (auto point : aligned_pc) {
    cgal::PointAndPrimitiveId ppid = reference_mesh_.getClosestPrimitive(point.x, point.y, point.z);
    double squared_distance = reference_mesh_.squaredDistance(cgal::Point(point.x, point.y, point.z));
    if (references_new.find(reference_mesh_.getFacetIndex(ppid.second)) != references_new.end() && sqrt(squared_distance) < 0.5) {
      ++point_count;
      result += sqrt(squared_distance);
    }
  }
  std::cout << "Approximation of ICP error to selected references is: " << result/point_count << std::endl;
  return result/point_count;
}

double Mapper::getICPError(const DP &aligned_dp) {
  PointCloud aligned_pc = cpt_utils::dpToPointCloud(aligned_dp);
  int point_count = 0;
  double result = 0;
  for (auto point : aligned_pc) {
    double squared_distance = reference_mesh_.squaredDistance(cgal::Point(point.x, point.y, point.z));
    if (sqrt(squared_distance) < 0.5) {
      ++point_count;
      result += sqrt(squared_distance);
    }
  }
  std::cout << "Approximation of ICP error to complete model is: " << result/point_count << std::endl;
  return result/point_count;
}

void Mapper::getError(DP ref, DP aligned_dp) {
  // https://github.com/ethz-asl/libpointmatcher/issues/193

  // reuse the same module used for the icp object is not possible with ICPSequence

  // https://github.com/ethz-asl/libpointmatcher/blob/master/examples/icp_advance_api.cpp
  // initiate matching with unfiltered point cloud

  // Generate new matcher module

  // Residual error: This is kind of bad since there is nearly never a 1:1 match of ref and reading in both directions
  const int knn = 1;
  PM::Parameters params;
  params["knn"] = PointMatcherSupport::toParam(knn);
  // other parameters possible

  std::shared_ptr<PM::Matcher> matcher = PM::get().MatcherRegistrar.create("KDTreeMatcher", params);
  matcher->init(ref);
  PM::Matches matches = matcher->findClosests(aligned_dp);
  // weight paired points
  PM::OutlierWeights outlierWeights = icp_.outlierFilters.compute(aligned_dp, ref, matches);
  // get error, why is error smaller if outlier filter ratio is smaller and result completely misaligned?!
  float error = icp_.errorMinimizer->getResidualError(aligned_dp, ref, outlierWeights, matches);
  std::cout << "Final residual error: " << error << std::endl;

  
}

// this is general pre-processing of dp cloud for map and reading cloud
void Mapper::processCloud(DP *point_cloud,
                          const ros::Time &stamp) {
  const size_t good_count(point_cloud->features.cols());
  if (good_count == 0) {
    ROS_ERROR("[ICP] I found no good points in the cloud");
    return;
  }

  // This need to be depreciated, there is addTime for those field in pm.
  if (!(point_cloud->descriptorExists("stamps_Msec")
      && point_cloud->descriptorExists("stamps_sec")
      && point_cloud->descriptorExists("stamps_nsec"))) {
    const float Msec = round(stamp.sec / 1e6);
    const float sec = round(stamp.sec - Msec * 1e6);
    const float nsec = round(stamp.nsec);

    const PM::Matrix desc_Msec = PM::Matrix::Constant(1, good_count, Msec);
    const PM::Matrix desc_sec = PM::Matrix::Constant(1, good_count, sec);
    const PM::Matrix desc_nsec = PM::Matrix::Constant(1, good_count, nsec);
    point_cloud->addDescriptor("stamps_Msec", desc_Msec);
    point_cloud->addDescriptor("stamps_sec", desc_sec);
    point_cloud->addDescriptor("stamps_nsec", desc_nsec);
  }

  int pts_count = point_cloud->getNbPoints();
  if (pts_count < parameters_.min_reading_point_count) {
    ROS_ERROR_STREAM(
        "[ICP] Not enough points in new PointCloud: only " << pts_count
                                                          << " pts.");
  }

  //TODO: remove if not needed (do it in icp config anyway)
  input_filters_.apply(*point_cloud);

  // Ensure a minimum amount of point after filtering
  pts_count = point_cloud->getNbPoints();
  if (pts_count < parameters_.min_reading_point_count) {
    ROS_ERROR_STREAM(
        "[ICP] Not enough points in newPointCloud: only " << pts_count
                                                          << " pts.");
  }
}

void Mapper::addScanToMap(DP &corrected_cloud, ros::Time &stamp) {
  // Preparation of cloud for inclusion in map
  map_pre_filters_.apply(corrected_cloud);
  // Merge cloud to map
  const size_t good_count(mapPointCloud.features.cols());
  if (good_count == 0) {
    mapPointCloud = corrected_cloud;
  }
  else {
    mapPointCloud.concatenate(corrected_cloud);
  }
  // Map maintenance
  map_post_filters_.apply(mapPointCloud);

  if (map_pub_.getNumSubscribers()) {
    map_pub_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapPointCloud,
                                                                        parameters_.tf_map_frame,
                                                                        stamp));
  }

  std::cout << "New map created" << std::endl;

  if (parameters_.update_icp_ref_trigger == true) {
    DP ref_pc = mapPointCloud; // add references 
    ref_pc.concatenate(selective_ref_dp);
    selective_icp_.clearMap();
    selective_icp_.setMap(ref_pc);
    std::cout << "New map set" << std::endl;
  }    
}

bool Mapper::setReferenceFacets(cpt_selective_icp::References::Request &req,
                                cpt_selective_icp::References::Response &res) {
  
  selective_icp_trigger = true; // use selective ICP now
  std::cout << "Mode set to selective ICP" << std::endl;

  std::unordered_set<int> references;
  for (auto id : req.data) {
    references.insert(id);
  }
  PointCloud pointcloud;
  extractReferenceFacets(parameters_.map_sampling_density, reference_mesh_, references, &pointcloud);

  selective_ref_dp = cpt_utils::pointCloudToDP(pointcloud);
  processCloud(&selective_ref_dp, ros::Time(0));

  // set the map
  selective_icp_.clearMap();
  selective_icp_.setMap(selective_ref_dp);

  // clear mapPointCloud
  DP new_cloud;
  mapPointCloud = new_cloud;

  return true;
}

bool Mapper::setSelectiveICP(std_srvs::SetBool::Request &req,
                             std_srvs::SetBool::Response &res) {
  if (req.data == true) {
    if (!selective_icp_.hasMap()) {
      std::cerr << "Please set reference facets first." << std::endl;
      res.success = false;
      return true;
    }
    selective_icp_trigger = true;
  } else {
    if (normal_icp_trigger == true)
      selective_icp_trigger = false;
    else
      std::cerr << "Can not turn both ICP methods off. Ignoring command." << std::endl;
  }
  res.success = true;
  return true;
}

bool Mapper::setNormalICP(std_srvs::SetBool::Request &req,
                          std_srvs::SetBool::Response &res) {
  
  if (req.data == true) {
    normal_icp_trigger = true;
    std::cout << "Mode set to normal ICP" << std::endl;

    // just called to publish ref mesh and p.c. again:
    std::unordered_set<int> references; // empty
    PointCloud pointcloud;
    extractReferenceFacets(parameters_.map_sampling_density, reference_mesh_, references, &pointcloud);
  } else {
    if (selective_icp_trigger == true) 
      normal_icp_trigger = false;
    else
      std::cerr << "Can not turn both ICP methods off. Ignoring command." << std::endl;
  }

  res.success = true;
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
    std::cout << "Mesh for ICP is sampled with " << n_points << " points" << std::endl;
    cpt_utils::sample_pc_from_mesh(reference_mesh.getMesh(), n_points, 0.0, pointcloud, "ref_pointcloud");
    publishReferenceMesh(reference_mesh, references);
  }
  else {
    std::cout << "Extract selected reference point clouds (selective ICP)" << std::endl;

    // sample reference point cloud from mesh

    // get coplanar facets
    std::cout << "Choosen reference facets:" << std::endl;
    std::unordered_set<int>::iterator uitr;
    for (uitr = references.begin(); uitr != references.end(); uitr++) {
      std::cout << *uitr << std::endl;
      reference_mesh.findCoplanarFacets(*uitr, &references_new, 0.01);
    }

    if (references_new.empty()) {
      return;
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
    std::cout << "Mesh for selective ICP is sampled with " << no_of_points << " points" << std::endl;
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

void Mapper::loadConfig() {
  // Load configs.

  // normal ICP
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

  // selective ICP
  if (ros::param::get("~selectiveIcpConfig", config_file_name)) {
    std::ifstream ifs(config_file_name.c_str());
    if (ifs.good()) {
      selective_icp_.loadFromYaml(ifs);
    } else {
      ROS_ERROR_STREAM(
          "Cannot load selective ICP config from YAML file " << config_file_name);
      selective_icp_.setDefault();
    }
  } else {
    ROS_INFO_STREAM("No selective ICP config file given, using default");
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

bool Mapper::reloadConfig(std_srvs::Empty::Request &req,
                          std_srvs::Empty::Response &res) {
  loadConfig();
  ROS_INFO_STREAM("Parameters reloaded");

  return true;
}

}
}
