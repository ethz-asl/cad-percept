#include "cpt_deviation_analysis/relative_deviations_ros.h"

namespace cad_percept {
namespace deviations {

RelativeDeviations::RelativeDeviations(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      map_frame_(nh_private.param<std::string>("map_frame", "fail")),
      discrete_color_(nh_private.param<bool>("discrete_color", false)),
      score_threshold_(nh_private.param<float>("score_threshold", 0.01)),
      cb(nh_private.param<int>("buffer_size", 1)),  // use x latest scans for detection
      cad_topic(nh_private.param<std::string>("cad_topic", "fail")),
      scan_topic(nh_private.param<std::string>("scan_topic", "fail")),
      map_topic(nh_private.param<std::string>("map_topic", "fail")),
      input_queue_size(nh_private.param<int>("inputQueueSize", 10)),
      visualize(nh_private.param<std::string>("visualize", "current")),
      map_analyzer_trigger(false) {
  // TODO (Hermann) rewrite to use config-provider
  // set parameters for Deviations object (public struct)
  deviations.params.planarSegmentation = nh_private.param<std::string>("planarSegmentation", "PCL");
  deviations.params.planarSegmentationMethod =
      nh_private.param<std::string>("planarSegmentationMethod", "RANSAC");
  deviations.params.path = nh_private.param<std::string>("path", "fail");
  deviations.params.segmentationDistanceThreshold =
      nh_private.param<double>("segmentationDistanceThreshold", 0.05);
  deviations.params.segmentationNormalThreshold =
      nh_private.param<double>("segmentationNormalThreshold", 0.9);
  deviations.params.segmentationClusterDistance =
      nh_private.param<double>("segmentationClusterDistance", 0.5);
  deviations.params.minNumberOfPlanePoints = nh_private.param<int>("minNumberOfPlanePoints", 50);
  deviations.params.segmentationProbability =
      nh_private.param<double>("segmentationProbability", 0.05);
  deviations.params.minPolyhedronArea = nh_private.param<double>("minPolyhedronArea", 0.05);
  deviations.params.matchScoreUpperLimit = nh_private.param<double>("matchScoreUpperLimit", 2.0);
  deviations.params.matchDistScoreThresh = nh_private.param<double>("matchDistScoreThresh", 1.0);
  deviations.params.matchMinDistThresh = nh_private.param<double>("matchMinDistThresh", 2.0);
  deviations.params.matchDistThresh = nh_private.param<double>("matchDistThresh", 2.0);
  deviations.params.matchAngleThresh = nh_private.param<double>("matchAngleThresh", 1.5);
  deviations.params.minDistWeight = nh_private.param<double>("minDistWeight", 0.0);
  deviations.params.distanceScoreWeight = nh_private.param<double>("distanceScoreWeight", 1.0);
  deviations.params.angleWeight = nh_private.param<double>("angleWeight", 0.0);
  deviations.params.distWeight = nh_private.param<double>("distWeight", 0.0);
  deviations.params.assocAreaRatioUpperLimit =
      nh_private.param<double>("assocAreaRatioUpperLimit", 10.0);
  deviations.params.assocAreaRatioLowerLimit =
      nh_private.param<double>("assocAreaRatioLowerLimit", 0.2);
  deviations.params.assocAreaLowerLimitThreshold =
      nh_private.param<double>("assocAreaLowerLimitThreshold", 0.5);

  buffer_pc_pub_ = nh_.advertise<PointCloud>("buffer_pc_pub", 1, true);
  reconstructed_planes_pub_ = nh_.advertise<ColoredPointCloud>("reconstructed_planes_pub", 1, true);
  polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("polygon_pub", 1, true);
  assoc_mesh_pub_ = nh_.advertise<cgal_msgs::ColoredMesh>("assoc_mesh", 1, true);
  assoc_pc_pub_ = nh_.advertise<ColoredPointCloud>("assoc_pc_pub", 1, true);
  assoc_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("assoc_marker_pub", 100, true);
  bboxes_marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("bboxes_marker_pub", 100, true);
  mesh_normals_marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("mesh_normals_marker_pub", 100, true);
  all_mesh_normals_marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("all_mesh_normals_marker_pub", 100, true);
  deviations_mesh_pub_ = nh_.advertise<cgal_msgs::ColoredMesh>("deviations_mesh_pub", 1, true);
  deviations_pub_ = nh_.advertise<cgal_msgs::GeomDeviation>("geometric_deviations", 1, true);

  analyze_map_srv_ = nh_.advertiseService("analyze_map", &RelativeDeviations::analyzeMap, this);
  set_deviation_plane_ = nh_.advertiseService(
      "set_deviation_target", &RelativeDeviations::deviationTargetServiceCallback, this);
  cad_sub_ = nh_.subscribe(cad_topic, input_queue_size, &RelativeDeviations::gotCAD, this);
  cloud_sub_ = nh_.subscribe(scan_topic, input_queue_size, &RelativeDeviations::gotCloud, this);
  map_sub_ = nh_.subscribe(map_topic, input_queue_size, &RelativeDeviations::gotMap, this);
}

void RelativeDeviations::gotCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in) {
  if (tf_listener_.canTransform(map_topic, cad_mesh_in.header.frame_id, ros::Time(0))) {
    tf::StampedTransform transform;
    tf_listener_.lookupTransform(map_topic, cad_mesh_in.header.frame_id, ros::Time(0), transform);
    cgal::MeshModel::Ptr model_ptr;
    cgal::msgToMeshModel(cad_mesh_in.mesh, &model_ptr);
    deviations.init(model_ptr, transform);
    publishAllModelNormals(deviations.plane_map);
  }
}

void RelativeDeviations::gotCloud(const sensor_msgs::PointCloud2 &cloud_msg_in) {
  std::cout << "[RD] Received new aligned PointCloud from selective ICP" << std::endl;
  PointCloud cloud;
  pcl::fromROSMsg(cloud_msg_in, cloud);
  processBuffer(cloud);
}

void RelativeDeviations::gotMap(const sensor_msgs::PointCloud2 &cloud_msg_in) {
  if (map_analyzer_trigger) {
    std::cout << "[RD] Received new aligned Map PointCloud from selective ICP" << std::endl;
    PointCloud cloud;
    pcl::fromROSMsg(cloud_msg_in, cloud);
    processMap(cloud);
    map_analyzer_trigger = false;
  }
}

bool RelativeDeviations::deviationTargetServiceCallback(
    cgal_msgs::SetDeviationPlane::Request &req, cgal_msgs::SetDeviationPlane::Response &resp) {
  selected_plane_ = deviations.facetToPlane[req.facet_id];
  std::cout << "Selected plane for facet " << req.facet_id << " is " << selected_plane_
            << std::endl;
  current_task_id_ = req.task_id;
  return true;
}

bool RelativeDeviations::analyzeMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  map_analyzer_trigger = true;
  return true;
}

void RelativeDeviations::processBuffer(PointCloud &reading_pc) {
  // insert reading_pc in buffer
  // the reading_pc should be pre-aligned with model here before continueing (from selective ICP)
  PointMatcherSupport::timer t_processBuffer;

  cb.push_back(reading_pc);
  if (cb.full()) {
    PointCloud::Ptr aligned_pc(new PointCloud());

    // concatenate point clouds in buffer
    for (auto pointcloud : cb) {
      *aligned_pc += pointcloud;
    }

    // downsample
    std::cout << "Size before filtering: " << aligned_pc->size() << std::endl;
    PointCloud cloud_filtered;
    pcl::RandomSample<pcl::PointXYZ> filter;
    filter.setInputCloud(aligned_pc);
    filter.setSample(aligned_pc->size() / cb.size());
    filter.filter(cloud_filtered);
    std::cout << "Size after filtering: " << cloud_filtered.size() << std::endl;

    // continue
    publishCloud<PointCloud>(&cloud_filtered, &buffer_pc_pub_);
    processCloud(cloud_filtered);
  }
}

void RelativeDeviations::processCloud(PointCloud &reading_pc) {
  std::vector<reconstructed_plane> rec_planes;
  std::vector<reconstructed_plane>
      remaining_plane_cloud_vector;  // put everything in here what can't be associated

  PointMatcherSupport::timer t_processCloud;

  deviations.detectChanges(&rec_planes, reading_pc, &remaining_plane_cloud_vector);

  if (visualize == "current") {
    publish(rec_planes, remaining_plane_cloud_vector, deviations.transformation_map);
  }

  /**
   *  Create geometric deviation messages and associated point cloud portions
   */

  std::cout << "selected plane " << selected_plane_ << " has "
            << deviations.transformation_map.count(selected_plane_) << " deviations." << std::endl;

  if (deviations.transformation_map.count(selected_plane_) > 0) {
    auto transform = deviations.transformation_map[selected_plane_];
    cgal_msgs::GeomDeviation deviation_msg;
    deviation_msg.element_id = selected_plane_;
    cpt_utils::toRosTransform(transform.translation, transform.quat,
                              &(deviation_msg.deviation_transform));
    pcl::toROSMsg(deviations.plane_map[selected_plane_].rec_plane.pointcloud,
                  deviation_msg.pointcloud);
    deviation_msg.pointcloud.header.frame_id = map_frame_;
    deviation_msg.pointcloud.header.stamp = ros::Time(0);
    deviations_pub_.publish(deviation_msg);
  }
  // reset here in case we still want to access something, otherwise can put in detectChanges
  deviations.reset();
}

void RelativeDeviations::processMap(PointCloud &map_pc) {
  std::vector<reconstructed_plane> rec_planes;
  std::vector<reconstructed_plane>
      remaining_plane_cloud_vector;  // put everything in here what we can't segment as planes
  std::unordered_map<std::string, transformation> current_transformation_map;

  PointMatcherSupport::timer t_processMap;

  deviations.detectMapChanges(&rec_planes, map_pc, &remaining_plane_cloud_vector,
                              &current_transformation_map);

  std::cout << "Transformation map has size: " << current_transformation_map.size() << std::endl;

  if (visualize == "map") {
    publish(rec_planes, remaining_plane_cloud_vector, current_transformation_map);
  }

  // reset here in case we still want to access something, otherwise can put in detectChanges
  deviations.reset();
}

void RelativeDeviations::publish(
    const std::vector<reconstructed_plane> &rec_planes,
    const std::vector<reconstructed_plane> &remaining_plane_cloud_vector,
    std::unordered_map<std::string, transformation> &transformation_map) {
  publishReconstructedPlanes(rec_planes, &reconstructed_planes_pub_);
  publishAssociations(deviations.reference_mesh, deviations.plane_map,
                      remaining_plane_cloud_vector);
  publishBboxesAndNormals(deviations.plane_map);
  publishModelNormals(deviations.plane_map);  // only of the associated ones
  publishDeviations(deviations.reference_mesh, transformation_map);
}

void RelativeDeviations::publishMesh(const cgal::MeshModel::Ptr &model,
                                     ros::Publisher *publisher) const {
  cgal_msgs::TriangleMesh t_msg;
  cgal_msgs::ColoredMesh c_msg;
  cgal::meshModelToMsg(model, &t_msg);
  c_msg.mesh = t_msg;

  std_msgs::ColorRGBA c;
  c.r = 0.0;
  c.g = 0.0;
  c.b = 1.0;
  c.a = 0.8;
  c_msg.color = c;

  c_msg.header.frame_id = map_frame_;
  c_msg.header.stamp = {secs : 0, nsecs : 0};
  c_msg.header.seq = 0;
  publisher->publish(c_msg);
}

template <class T>
void RelativeDeviations::publishCloud(T *cloud, ros::Publisher *publisher) const {
  cloud->header.frame_id = map_frame_;
  pcl_conversions::toPCL(ros::Time(0), cloud->header.stamp);
  publisher->publish(*cloud);
}

void RelativeDeviations::publishPolyhedron(cgal::Polyhedron &P) {
  // use this as long there is no rviz Polyhedron Display for non-triangles
  for (cgal::Polyhedron::Facet_iterator j = P.facets_begin(); j != P.facets_end(); ++j) {
    // TODO (Hermann) Can we remove this?
    sleep(5);
    std::cout << "Publish Polyhedron" << std::endl;
    geometry_msgs::PolygonStamped msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = ros::Time(0);
    cgal::Polyhedron::Halfedge_around_facet_const_circulator hit = j->facet_begin();

    do {
      geometry_msgs::Point32 p;
      p.x = (float)(hit->vertex()->point()).x();
      p.y = (float)(hit->vertex()->point()).y();
      p.z = (float)(hit->vertex()->point()).z();
      msg.polygon.points.push_back(p);
    } while (++hit != j->facet_begin());

    polygon_pub_.publish(msg);
  }
}

void RelativeDeviations::publishReconstructedPlanes(
    const std::vector<reconstructed_plane> &rec_planes, ros::Publisher *publisher) const {
  ColoredPointCloud pointcloud_rgb;
  for (auto plane : rec_planes) {
    ColoredPointCloud pointcloud_plane_rgb;
    pcl::copyPointCloud(plane.pointcloud, pointcloud_plane_rgb);
    uint8_t r = std::rand() % 256, g = std::rand() % 256, b = std::rand() % 256;
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    for (uint i = 0; i < pointcloud_plane_rgb.points.size(); ++i) {
      pointcloud_plane_rgb.points[i].rgb = *reinterpret_cast<float *>(&rgb);
    }
    pointcloud_rgb += pointcloud_plane_rgb;
  }
  publishCloud<ColoredPointCloud>(&pointcloud_rgb, publisher);
}

void RelativeDeviations::publishAssociations(
    const cgal::MeshModel::Ptr &model, std::unordered_map<std::string, polyhedron_plane> &plane_map,
    const std::vector<reconstructed_plane> &remaining_plane_cloud_vector) {
  ColoredPointCloud pointcloud_rgb;
  ColoredPointCloud pointcloud_plane_rgb;

  cgal_msgs::TriangleMesh t_msg;
  cgal_msgs::ColoredMesh c_msg;
  cgal::meshModelToMsg(model, &t_msg);
  c_msg.mesh = t_msg;
  c_msg.header.frame_id = map_frame_;
  c_msg.header.stamp = {secs : 0, nsecs : 0};
  c_msg.header.seq = 0;

  /**
   *  Non-associated planes PointCloud
   */
  // add non associated rec. planes in blue
  for (auto plane : remaining_plane_cloud_vector) {
    pcl::copyPointCloud(plane.pointcloud, pointcloud_plane_rgb);
    uint8_t r = 0, g = 0, b = 255;
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    for (uint i = 0; i < pointcloud_plane_rgb.points.size(); ++i) {
      pointcloud_plane_rgb.points[i].rgb = *reinterpret_cast<float *>(&rgb);
    }
    pointcloud_rgb += pointcloud_plane_rgb;
  }

  /**
   *  Associated Triangles, Rec. Planes P.C.
   */
  std_msgs::ColorRGBA c;

  // set all facet colors to blue, non-associated stay blue
  for (uint i = 0; i < c_msg.mesh.triangles.size(); ++i) {
    c.r = 0.0;
    c.g = 0.0;
    c.b = 1.0;
    c.a = 0.4;
    c_msg.colors.push_back(c);
  }

  // overwrite color of associated planes/triangles
  visualization_msgs::MarkerArray marker_array;
  int marker_id = 0;
  for (auto &[plane_id, plane] : plane_map) {
    if (plane.associated) {
      uint8_t r = std::rand() % 256, g = std::rand() % 256, b = 0;

      auto triangle_range = deviations.planeToFacets.equal_range(plane_id);
      for (auto triangle_it = triangle_range.first; triangle_it != triangle_range.second;
           ++triangle_it) {
        c.r = r / 255.;
        c.g = g / 255.;
        c.b = b / 255.;
        c.a = 0.4;
        // find index of triangle with this id
        // TODO (Hermann) this is not really efficient, maybe we can get a lookup back when creating
        // the message?
        std::vector<std::string>::iterator id_it = std::find(
            c_msg.mesh.triangle_ids.begin(), c_msg.mesh.triangle_ids.end(), triangle_it->second);
        if (id_it != c_msg.mesh.triangle_ids.end()) {
          size_t index = std::distance(c_msg.mesh.triangle_ids.begin(), id_it);
          c_msg.colors[index] = c;
        }
        // keep color association for other functions
        c_associated_[plane_id] = c;
      }

      pcl::copyPointCloud(plane.rec_plane.pointcloud, pointcloud_plane_rgb);
      uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
      for (uint i = 0; i < pointcloud_plane_rgb.points.size(); ++i) {
        pointcloud_plane_rgb.points[i].rgb = *reinterpret_cast<float *>(&rgb);
      }
      pointcloud_rgb += pointcloud_plane_rgb;

      // marker connecting points and polyhedrons in corresponding random color
      visualization_msgs::Marker marker;
      marker.header.frame_id = map_frame_;
      marker.ns = "semantic_graph_matches";
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.id = marker_id;
      marker.scale.x = 0.01f;
      marker.color.r = r / 255.;
      marker.color.g = g / 255.;
      marker.color.b = b / 255.;
      marker.color.a = 0.7f;
      geometry_msgs::Point p_from, p_to;
      for (auto point : plane.rec_plane.pointcloud) {
        p_from.x = point.x;
        p_from.y = point.y;
        p_from.z = point.z;
        cgal::Point p =
            cpt_utils::closestPointOnPlane(plane.plane, cgal::Point(point.x, point.y, point.z));
        p_to.x = p.x();
        p_to.y = p.y();
        p_to.z = p.z();
        marker.points.push_back(p_from);
        marker.points.push_back(p_to);
      }
      marker_array.markers.push_back(marker);
    }
    ++marker_id;
  }

  publishCloud<ColoredPointCloud>(&pointcloud_rgb, &assoc_pc_pub_);
  // TODO: Backface culling should be turned on, but is somehow not activated after new msg
  assoc_mesh_pub_.publish(c_msg);
  assoc_marker_pub_.publish(marker_array);
}

// of pc
void RelativeDeviations::publishBboxesAndNormals(
    std::unordered_map<std::string, polyhedron_plane> &plane_map) {
  visualization_msgs::MarkerArray marker_array;
  int marker_id = 0;
  for (auto &[plane_id, plane] : plane_map) {
    if (plane.associated) {
      /**
       *  BBox
       */
      visualization_msgs::Marker marker;
      marker.header.frame_id = map_frame_;
      marker.ns = "bboxes";
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.id = marker_id;
      // get color of one of the corresponding facets
      marker.color = c_associated_[plane_id];
      marker.color.a = 0.7f;

      CGAL::Bbox_3 bbox;
      cpt_utils::computePCBbox(plane.rec_plane.pointcloud, &bbox);
      cgal::Point center = cpt_utils::centerOfBbox(bbox);
      marker.pose.position.x = center.x();
      marker.pose.position.y = center.y();
      marker.pose.position.z = center.z();
      marker.scale.x = bbox.xmax() - bbox.xmin();
      marker.scale.y = bbox.ymax() - bbox.ymin();
      marker.scale.z = bbox.zmax() - bbox.zmin();

      marker_array.markers.push_back(marker);

      /**
       *  Normal
       */
      visualization_msgs::Marker marker_2;
      marker_2.header.frame_id = map_frame_;
      marker_2.ns = "normals";
      marker_2.type = visualization_msgs::Marker::ARROW;
      marker_2.action = visualization_msgs::Marker::ADD;
      marker_2.id = marker_id;
      marker_2.color = c_associated_[plane_id];
      marker_2.color.a = 0.7f;
      marker_2.scale.x = 0.05;
      marker_2.scale.y = 0.1;

      geometry_msgs::Point p_from, p_to;
      p_from.x = center.x();
      p_from.y = center.y();
      p_from.z = center.z();
      p_to.x = center.x() + plane.rec_plane.pc_normal(0);
      p_to.y = center.y() + plane.rec_plane.pc_normal(1);
      p_to.z = center.z() + plane.rec_plane.pc_normal(2);
      marker_2.points.push_back(p_from);
      marker_2.points.push_back(p_to);

      marker_array.markers.push_back(marker_2);

      ++marker_id;
    }
  }
  bboxes_marker_pub_.publish(marker_array);
}

void RelativeDeviations::publishModelNormals(
    std::unordered_map<std::string, polyhedron_plane> &plane_map) {
  visualization_msgs::MarkerArray marker_array;
  int marker_id = 0;
  for (auto &[plane_id, plane] : plane_map) {
    if (plane.associated) {
      cgal::Point center = cpt_utils::centerOfBbox(plane.bbox);

      visualization_msgs::Marker marker;
      marker.header.frame_id = map_frame_;
      marker.ns = "mesh_normals";
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.id = marker_id;
      marker.color.r = 0;
      marker.color.g = 0;
      marker.color.b = 1;
      marker.color.a = 0.7f;
      marker.scale.x = 0.05;
      marker.scale.y = 0.1;

      geometry_msgs::Point p_from, p_to;
      p_from.x = center.x();
      p_from.y = center.y();
      p_from.z = center.z();
      p_to.x = center.x() + plane.normal(0);
      p_to.y = center.y() + plane.normal(1);
      p_to.z = center.z() + plane.normal(2);
      marker.points.push_back(p_from);
      marker.points.push_back(p_to);

      marker_array.markers.push_back(marker);

      ++marker_id;
    }
  }
  mesh_normals_marker_pub_.publish(marker_array);
}

void RelativeDeviations::publishAllModelNormals(
    std::unordered_map<std::string, polyhedron_plane> &plane_map) {
  visualization_msgs::MarkerArray marker_array;
  int marker_id = 0;
  for (auto &[plane_id, plane] : plane_map) {
    cgal::Point center = cpt_utils::centerOfBbox(plane.bbox);

    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.ns = "total_mesh_normals";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = marker_id;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.color.a = 0.7f;
    marker.scale.x = 0.05;
    marker.scale.y = 0.1;

    geometry_msgs::Point p_from, p_to;
    p_from.x = center.x();
    p_from.y = center.y();
    p_from.z = center.z();
    p_to.x = center.x() + plane.normal(0);
    p_to.y = center.y() + plane.normal(1);
    p_to.z = center.z() + plane.normal(2);
    marker.points.push_back(p_from);
    marker.points.push_back(p_to);

    marker_array.markers.push_back(marker);

    ++marker_id;
  }
  all_mesh_normals_marker_pub_.publish(marker_array);
}

void RelativeDeviations::publishDeviations(
    const cgal::MeshModel::Ptr &model,
    std::unordered_map<std::string, transformation> &transformation_map) {
  cgal_msgs::TriangleMesh t_msg;
  cgal_msgs::ColoredMesh c_msg;
  cgal::meshModelToMsg(model, &t_msg);
  c_msg.mesh = t_msg;
  c_msg.header.frame_id = map_frame_;
  c_msg.header.stamp = {secs : 0, nsecs : 0};
  c_msg.header.seq = 0;

  std_msgs::ColorRGBA c;

  // set all facet colors to blue, non-associated stay blue
  for (uint i = 0; i < c_msg.mesh.triangles.size(); ++i) {
    c.r = 0.0;
    c.g = 0.0;
    c.b = 1.0;
    c.a = 0.4;
    c_msg.colors.push_back(c);
  }

  // - overwrite color of associated planes/triangles with deviation gradient based on e.g.
  // distance score, because distance score includes both angle deviation and distance deviation
  // - set threshold for ok walls
  for (auto &[plane_id, transformation] : transformation_map) {
    double score = transformation.score;

    if (score < score_threshold_) {
      c.r = 0.0;
      c.g = 1.0;
      c.b = 0.0;
      c.a = 0.4;
    } else {
      // make gradient based on score here
      if (discrete_color_ == true) {
        c.r = 1.0;
        c.g = 0.0;
        c.b = 0.0;
        c.a = 0.4;
      } else {
        if (score > 0.12) {
          c.r = 1.0;
          c.g = 0.0;
          c.b = 0.0;
          c.a = 0.4;
        } else {
          // create a gradient
          float g = score / 0.12;  // 1 for red, 0 for green
          if (g > 0.5) {
            c.r = 1.0;
            c.g = 2.0 * (1 - g);
          } else {
            c.r = 2 * g;
            c.g = 1.0;
          }
          c.b = 0.0;
          c.a = 0.4;
        }
      }
    }

    auto triangle_range = deviations.planeToFacets.equal_range(plane_id);
    for (auto triangle_it = triangle_range.first; triangle_it != triangle_range.second;
         ++triangle_it) {
      // find index of triangle with this id
      // TODO (Hermann) this is not really efficient, maybe we can get a lookup back when creating
      // the message?
      std::vector<std::string>::iterator id_it = std::find(
          c_msg.mesh.triangle_ids.begin(), c_msg.mesh.triangle_ids.end(), triangle_it->second);
      if (id_it != c_msg.mesh.triangle_ids.end()) {
        size_t index = std::distance(c_msg.mesh.triangle_ids.begin(), id_it);
        c_msg.colors[index] = c;
      }
    }
  }
  deviations_mesh_pub_.publish(c_msg);
}

}  // namespace deviations
}  // namespace cad_percept
