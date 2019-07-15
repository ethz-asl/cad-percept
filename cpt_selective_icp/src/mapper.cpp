#include "cpt_selective_icp/mapper.h"

namespace cad_percept {
namespace mapper {

Mapper::Mapper(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
  : nh_(nh),
    nh_private_(nh_private),
    tf_listener_(ros::Duration(30)),
    odom_received_(0),
    T_scanner_to_map_(PM::TransformationParameters::Identity(4, 4)),
    min_reading_point_count(getParam<int>("minReadingPointCount", 2000)),
    tf_map_frame(getParam<std::string>("tf_map_frame", "map")),
    lidar_frame(getParam<std::string>("lidar_frame", "lidar")) {

  loadExternalParameters();
  cloud_sub_ = nh_.subscribe("cloud_in",
                        parameters_.input_queue_size,
                        &Mapper::gotCloud,
                        this);
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

void Mapper::processCloud(unique_ptr<DP> new_point_cloud,
                          const std::string &scanner_frame,
                          const ros::Time &stamp,
                          uint32_t seq, bool new_map) {
  timer t;

  const size_t good_count(new_point_cloud->features.cols());
  if (good_count == 0) {
    ROS_ERROR("[ICP] I found no good points in the cloud");
    return;
  }

  // Dimension of the point cloud, important since we handle 2D and 3D.
  const int dimp1(new_point_cloud->features.rows());

  // This need to be depreciated, there is addTime for those field in pm.
  if (!(new_point_cloud->descriptorExists("stamps_Msec")
      && new_point_cloud->descriptorExists("stamps_sec")
      && new_point_cloud->descriptorExists("stamps_nsec"))) {
    const float Msec = round(stamp.sec / 1e6);
    const float sec = round(stamp.sec - Msec * 1e6);
    const float nsec = round(stamp.nsec);

    const PM::Matrix desc_Msec = PM::Matrix::Constant(1, good_count, Msec);
    const PM::Matrix desc_sec = PM::Matrix::Constant(1, good_count, sec);
    const PM::Matrix desc_nsec = PM::Matrix::Constant(1, good_count, nsec);
    new_point_cloud->addDescriptor("stamps_Msec", desc_Msec);
    new_point_cloud->addDescriptor("stamps_sec", desc_sec);
    new_point_cloud->addDescriptor("stamps_nsec", desc_nsec);
  }

  int pts_count = new_point_cloud->getNbPoints();
  if (pts_count < min_reading_point_count) {
    ROS_ERROR_STREAM(
        "[ICP] Not enough points in newPointCloud: only " << pts_count
                                                          << " pts.");
    return;
  }

  input_filters_.apply(*new_point_cloud);

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
                                                             - stamp << endl
                                                         << e.what());
    return;
  } catch (...) {
    // Everything else.
    ROS_ERROR_STREAM("Unexpected exception... ignoring scan.");
    return;
  }
  ROS_DEBUG_STREAM(
      "[ICP] T_scanner_to_map (" << scanner_frame << " to "
                                 << parameters_.map_frame << "):\n"
                                 << T_scanner_to_map_);

  const PM::TransformationParameters T_scanner_to_local_map =
      transformation_->correctParameters(
          T_local_map_to_map_.inverse() * T_scanner_to_map_);

  pts_count = new_point_cloud->getNbPoints();
  if (pts_count < parameters_.min_reading_point_count) {
    ROS_ERROR_STREAM(
        "[ICP] Not enough points in newPointCloud: only " << pts_count
                                                          << " pts.");
    return;
  }

  if (!icp_.hasMap() || new_map) {
    ROS_INFO_STREAM("[MAP] Creating an initial map");
    map_creation_time_ = stamp;
    setMap(updateMap(new_point_cloud.release(), T_scanner_to_map_, false));
    parameters_.map_trigger = false;
    return;
  }

  if (new_point_cloud->getEuclideanDim()
      != icp_.getPrefilteredInternalMap().getEuclideanDim()) {
    ROS_ERROR_STREAM("[ICP] Dimensionality missmatch: incoming cloud is "
                         << new_point_cloud->getEuclideanDim()
                         << " while map is "
                         << icp_.getPrefilteredInternalMap().getEuclideanDim());
    return;
  }

  try {
    PM::TransformationParameters T_updated_scanner_to_map;
    PM::TransformationParameters T_updated_scanner_to_local_map;

    ROS_DEBUG_STREAM(
        "[ICP] Computing - reading: " << new_point_cloud->getNbPoints()
                                      << ", reference: "
                                      << icp_.getPrefilteredInternalMap()
                                          .getNbPoints());

    icp_map_lock_.lock();
    T_updated_scanner_to_local_map = icp_(*new_point_cloud,
                                          T_scanner_to_local_map);
    icp_map_lock_.unlock();

    T_updated_scanner_to_map = T_local_map_to_map_ *
        T_updated_scanner_to_local_map;

    ROS_DEBUG_STREAM(
        "[ICP] T_updatedScanner_to_map:\n" << T_updated_scanner_to_map);
    ROS_DEBUG_STREAM("[ICP] T_updatedScanner_to_localMap:\n"
                         << T_updated_scanner_to_local_map);

    // Ensure minimum overlap between scans.
    const double estimated_overlap = icp_.errorMinimizer->getOverlap();
    ROS_DEBUG_STREAM("[ICP] Overlap: " << estimated_overlap);
    if (estimated_overlap < parameters_.min_overlap) {
      ROS_ERROR_STREAM(
          "[ICP] Estimated overlap too small, ignoring ICP correction!");
      return;
    }

    // Publish odometry.
    if (odom_pub_.getNumSubscribers()) {
      odom_pub_.publish(PointMatcher_ros::eigenMatrixToOdomMsg<float>(
          T_updated_scanner_to_map,
          parameters_.tf_map_frame,
          stamp));
    }
    // Publish pose.
    if (pose_pub_.getNumSubscribers()) {
      pose_pub_.publish(PointMatcher_ros::eigenMatrixToTransformStamped<float>(
          T_updated_scanner_to_map,
          parameters_.lidar_frame,
          parameters_.tf_map_frame,
          stamp));
    }
    if (map_pub_.getNumSubscribers()) {
      map_pub_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>
                           (*map_point_cloud_,
                            parameters_.tf_map_frame,
                            map_creation_time_));
    }
    // Publish the corrected scan point cloud
    DP pc = transformation_->compute(*new_point_cloud,
                                     T_updated_scanner_to_map);
    map_post_filters_.apply(pc);
    publish_lock_.lock();
    if (scan_pub_.getNumSubscribers() && parameters_.localizing) {
      ROS_DEBUG_STREAM(
          "Corrected scan publishing " << pc.getNbPoints() << " points");
      scan_pub_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(pc,
                                                                           parameters_.tf_map_frame,
                                                                           stamp));
    }
    publish_lock_.unlock();

    if (
        ((estimated_overlap < parameters_.max_overlap_to_merge)
            || (icp_.getPrefilteredInternalMap().features.cols()
                < parameters_.min_map_point_count)) &&
            (!map_building_in_progress_)
        ) {
      // Make sure we process the last available map.
      map_creation_time_ = stamp;

      ROS_DEBUG_STREAM("[MAP] Adding new points in a separate thread");

      map_building_task_ = MapBuildingTask(boost::bind(&Mapper::updateMap,
                                                       this,
                                                       new_point_cloud.release(),
                                                       T_updated_scanner_to_map,
                                                       true));
      map_building_future_ = map_building_task_.get_future();
      map_building_thread_ =
          boost::thread(boost::move(boost::ref(map_building_task_)));
      map_building_thread_.detach(); // We don't care about joining this one.
      sched_yield();
      map_building_in_progress_ = true;
    } else {
      cerr << "SKIPPING MAP" << endl;
      cerr << "estimatedOverlap < maxOverlapToMerge: "
           << (estimated_overlap < parameters_.max_overlap_to_merge) << endl;
      cerr
          << "(icp.getPrefilteredInternalMap().features.cols() < minMapPointCount): "
          << icp_.getPrefilteredInternalMap().features.cols() << " < "
          << parameters_.min_map_point_count
          << " = " << (icp_.getPrefilteredInternalMap().features.cols()
          < parameters_.min_map_point_count)
          << endl;
      cerr << "mapBuildingInProgress: " << map_building_in_progress_ << endl;

      bool state_lock = publish_lock_.try_lock();
      if (state_lock) publish_lock_.unlock();
      cerr << "publishLock.try_lock(): " << state_lock << endl;

      state_lock = icp_map_lock_.try_lock();
      if (state_lock) icp_map_lock_.unlock();
      cerr << "icpMapLock.try_lock(): " << state_lock << endl;

      cerr << "mapBuildingFuture.has_value(): " << map_building_future_
          .has_value()
           << endl;

    }

  } catch (PM::ConvergenceError error) {
    icp_map_lock_.unlock();
    ROS_ERROR_STREAM("[ICP] failed to converge: " << error.what());
    new_point_cloud->save("error_read.vtk");
    icp_.getPrefilteredMap().save("error_ref.vtk");
    return;
  } catch (...) {
    // everything else.
    publish_lock_.unlock();
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


void Mapper::loadExternalParameters() {
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

bool Mapper::reloadallYaml(std_srvs::Empty::Request &req,
                           std_srvs::Empty::Response &res) {
  loadExternalParameters();
  ROS_INFO_STREAM("Parameters reloaded");

  return true;
}

}
}