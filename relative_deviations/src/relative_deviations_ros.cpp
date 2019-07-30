#include "relative_deviations/relative_deviations_ros.h"

namespace cad_percept {
namespace deviations {

RelativeDeviations::RelativeDeviations(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
  : nh_(nh),
    nh_private_(nh_private),
    map_frame_(nh_private.param<std::string>("map_frame", "fail")),
    discrete_color_(nh_private.param<bool>("discrete_color", false)),
    score_threshold_(nh_private.param<float>("score_threshold", 0.01)),
    cb(10) { // use 10 latest scans for detection

  ref_mesh_pub_ = nh_.advertise<cgal_msgs::ColoredMesh>("ref_mesh", 1, true); // latching to true
  reading_pc_pub_ = nh_.advertise<PointCloud>("reading_pc_pub", 1, true);
  icp_pc_pub_ = nh_.advertise<PointCloud>("icp_pc_pub", 1, true);
  reconstructed_planes_pub_ = nh_.advertise<ColoredPointCloud>("reconstructed_planes_pub", 1, true);
  polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("polygon_pub", 1, true);
  assoc_mesh_pub_ = nh_.advertise<cgal_msgs::ColoredMesh>("assoc_mesh", 1, true); 
  assoc_pc_pub_ = nh_.advertise<ColoredPointCloud>("assoc_pc_pub", 1, true);
  assoc_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("assoc_marker_pub", 100, true);
  deviations_mesh_pub_ = nh_.advertise<cgal_msgs::ColoredMesh>("deviations_mesh_pub", 1, true);

  // manually starting test case
  if (nh_private_.param<bool>("test", "fail") == 1) {
    cgal::PointCloud reading_pc;
    createTestCase(&reading_pc);
    deviations.init(nh_private_.param<std::string>("reference_model_file", "fail").c_str());
    readingCallback(reading_pc);
  }
  else {
    deviations.init(nh_private_.param<std::string>("reference_model_file", "fail").c_str());
  }
}

RelativeDeviations::~RelativeDeviations() {}

void RelativeDeviations::createTestCase(cgal::PointCloud *reading_pc) {
  cgal::Polyhedron P;
  cgal::Polyhedron P_deviated;
  cgal::build_sample_polyhedrons(&P, &P_deviated);
  cgal::sample_pc_from_mesh(P_deviated, 3000, 0.01, reading_pc, "reading_pc"); 

  // transform reading pointcloud a little bit to test ICP
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << 0.5, 0.1, 0.2;
  float theta = M_PI*0.01;
  transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  cpt_utils::transformPointCloud(reading_pc, transform);
  pcl::io::savePCDFileASCII("/home/julian/cadify_ws/src/cad-percept/relative_deviations/resources/deviated_reading_pc.pcd", *reading_pc);
}

void RelativeDeviations::readingCallback(cgal::PointCloud &reading_pc) {
  publishMesh(deviations.reference_mesh, &ref_mesh_pub_);
  std::vector<reconstructed_plane> rec_planes;
  PointCloud icp_cloud;
  std::ifstream ifs_icp_config(nh_private_.param<std::string>("icp_configuration_file", "fail").c_str());
  std::ifstream ifs_selective_icp_config(nh_private_.param<std::string>("selective_icp_configuration_file", "fail").c_str());
  std::ifstream ifs_normal_filter(nh_private_.param<std::string>("normal_filter_file", "fail").c_str());
  std::vector<reconstructed_plane> remaining_cloud_vector;
  std::unordered_map<int, transformation> transformation_map;
  deviations.detectChanges(&rec_planes, reading_pc, &icp_cloud, ifs_icp_config, ifs_normal_filter, ifs_selective_icp_config, &remaining_cloud_vector, &transformation_map);
  publishReconstructedPlanes(rec_planes, &reconstructed_planes_pub_); 
  
  //cgal::Polyhedron P = deviations.reference_mesh_merged.getMesh();
  //publishPolyhedron(P);

  publishCloud<PointCloud>(&reading_pc, &reading_pc_pub_);
  publishCloud<PointCloud>(&icp_cloud, &icp_pc_pub_);
  publishAssociations(deviations.reference_mesh, deviations.plane_map, remaining_cloud_vector);
  publishDeviations(deviations.reference_mesh, deviations.plane_map, transformation_map);
  deviations.reset();
}

void RelativeDeviations::bufferCallback(cgal::PointCloud &reading_pc) {
  // insert reading_pc in buffer
  // the reading_pc should be pre-aligned with model here before continueing
  cb.push_back(reading_pc);
  if (cb.full()) {
    cgal::PointCloud aligned_pc;
    cpt_utils::align_sequence(cb, &aligned_pc);
    readingCallback(aligned_pc);
  }
}

void RelativeDeviations::publishMesh(const cgal::MeshModel &model, ros::Publisher *publisher) const {
  cgal::Polyhedron P;
  P = model.getMesh();
  cgal_msgs::TriangleMesh t_msg;
  cgal_msgs::ColoredMesh c_msg;
  cgal::triangleMeshToMsg(P, &t_msg);
  c_msg.mesh = t_msg;

  std_msgs::ColorRGBA c;
  c.r = 0.0;
  c.g = 0.0;
  c.b = 1.0;
  c.a = 0.8;
  c_msg.color = c;

  c_msg.header.frame_id = map_frame_;
  c_msg.header.stamp = {secs: 0, nsecs: 0};
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

void RelativeDeviations::publishReconstructedPlanes(const std::vector<reconstructed_plane> &rec_planes, ros::Publisher *publisher) const {
  ColoredPointCloud pointcloud_rgb;
  for (auto plane : rec_planes) {
    ColoredPointCloud pointcloud_plane_rgb;
    pcl::copyPointCloud(plane.pointcloud, pointcloud_plane_rgb);
    uint8_t r = std::rand()%256, g = std::rand()%256, b = std::rand()%256;    
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    for (uint i = 0; i < pointcloud_plane_rgb.points.size(); ++i) {
      pointcloud_plane_rgb.points[i].rgb = *reinterpret_cast<float*>(&rgb);
    }
    pointcloud_rgb += pointcloud_plane_rgb;
  }
  publishCloud<ColoredPointCloud>(&pointcloud_rgb, publisher);
}

void RelativeDeviations::publishAssociations(const cgal::MeshModel &model, std::unordered_map<int, polyhedron_plane> &plane_map, const std::vector<reconstructed_plane> &remaining_cloud_vector) {
  ColoredPointCloud pointcloud_rgb;
  ColoredPointCloud pointcloud_plane_rgb;
  
  cgal::Polyhedron P;
  P = model.getMesh();
  cgal_msgs::TriangleMesh t_msg;
  cgal_msgs::ColoredMesh c_msg;
  cgal::triangleMeshToMsg(P, &t_msg);
  c_msg.mesh = t_msg;
  c_msg.header.frame_id = map_frame_;
  c_msg.header.stamp = {secs: 0, nsecs: 0};
  c_msg.header.seq = 0;

  // add non associated rec. planes in blue
  for (auto plane : remaining_cloud_vector) {
    pcl::copyPointCloud(plane.pointcloud, pointcloud_plane_rgb);
    uint8_t r = 0, g = 0, b = 255;    
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    for (uint i = 0; i < pointcloud_plane_rgb.points.size(); ++i) {
      pointcloud_plane_rgb.points[i].rgb = *reinterpret_cast<float*>(&rgb);
    }
    pointcloud_rgb += pointcloud_plane_rgb;
  }

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
  for (Umiterator umit = plane_map.begin(); umit != plane_map.end(); ++umit) {
    if (umit->second.match_score != 0) {
      std::cout << "Visualize Facet: " << umit->first << std::endl;
      uint8_t r = std::rand()%256, g = std::rand()%256, b = 0;   

      auto iit = deviations.bimap.right.equal_range(umit->first);
      for (auto itr = iit.first; itr != iit.second; ++itr) {
        c.r = r/255.;
        c.g = g/255.;
        c.b = b/255.;
        c.a = 0.4;
        c_msg.colors[itr->second] = c;
      }

      pcl::copyPointCloud(umit->second.rec_plane.pointcloud, pointcloud_plane_rgb);
      uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
      for (uint i = 0; i < pointcloud_plane_rgb.points.size(); ++i) {
        pointcloud_plane_rgb.points[i].rgb = *reinterpret_cast<float*>(&rgb);      
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
      marker.color.r = r/255.;
      marker.color.g = g/255.;
      marker.color.b = b/255.;
      marker.color.a = 0.7f;
      geometry_msgs::Point p_from, p_to;
      for (auto point : umit->second.rec_plane.pointcloud) {
        p_from.x = point.x;
        p_from.y = point.y;
        p_from.z = point.z;
        cgal::Point p = cpt_utils::closestPointOnPlane(umit->second.plane, cgal::Point(point.x, point.y, point.z));
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
  //TODO: Backface culling should be turned on, but is somehow not activated after new msg
  assoc_mesh_pub_.publish(c_msg);
  assoc_marker_pub_.publish(marker_array);
}

void RelativeDeviations::publishDeviations(const cgal::MeshModel &model, std::unordered_map<int, polyhedron_plane> &plane_map, std::unordered_map<int, transformation> &transformation_map) {
  cgal::Polyhedron P;
  P = model.getMesh();
  cgal_msgs::TriangleMesh t_msg;
  cgal_msgs::ColoredMesh c_msg;
  cgal::triangleMeshToMsg(P, &t_msg);
  c_msg.mesh = t_msg;
  c_msg.header.frame_id = map_frame_;
  c_msg.header.stamp = {secs: 0, nsecs: 0};
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

  // - overwrite color of associated planes/triangles with deviation gradient based on e.g. distance score,
  // because distance score includes both angle deviation and distance deviation
  // - set threshold for ok walls

  for (Umiterator umit = plane_map.begin(); umit != plane_map.end(); ++umit) {
    if (umit->second.match_score != 0) {
      std::cout << "Visualize Deviation of Facet: " << umit->first << std::endl;
      
      //std::unordered_map<int,transformation>::const_iterator it = transformation_map.find(umit->first);
      transformation trafo = transformation_map[umit->first];
      double score = trafo.distance_score;

      if (score < score_threshold_) {
        c.r = 0.0;
        c.g = 1.0;
        c.b = 0.0;
        c.a = 0.4;   
      }
      else {
        // make gradient based on score here
        if (discrete_color_ == true) {
          c.r = 1.0;
          c.g = 0.0;
          c.b = 0.0;
          c.a = 0.4;
        }
        else {
          if (score > 0.75) {
            c.r = 1.0;
            c.g = 0.0;
            c.b = 0.0;
            c.a = 0.4;
          }
          else {
            // create a gradient
            float g = score/0.75; // 1 for red, 0 for green
            if (g > 0.5) {
              c.r = 1.0;
              c.g = 2.0 * (1 - g);
            } else {
              c.r = 2*g;
              c.g = 1.0;
            }
            c.b = 0.0;
            c.a = 0.4;
          }
        }
      }

      auto iit = deviations.bimap.right.equal_range(umit->first);
      for (auto itr = iit.first; itr != iit.second; ++itr) {
        c_msg.colors[itr->second] = c;
      }
    }
  }
  deviations_mesh_pub_.publish(c_msg);
}

}
}
