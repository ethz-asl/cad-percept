#include <cgal_visualizations/mesh_visual.h>

#include <chrono>
#include <limits>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <ros/package.h>  // This dependency should be moved out of here, it is just used for a search path.
#include <ros/ros.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/bool_property.h>

namespace cad_percept {

namespace visualizations {

unsigned int MeshVisual::instance_counter_ = 0;

MeshVisual::MeshVisual(Ogre::SceneManager* scene_manager,
                       Ogre::SceneNode* parent_node)
    : visualize_covariances_(false),
      backface_culling_(false),
      visualize_color_(false),
      std_edge_color_(0.0, 0.0, 0.0, 0.8),
      std_surface_color_(0.0, 0.0, 1.0, 0.8) {
  scene_manager_ = scene_manager;
  frame_node_ = parent_node;
  instance_number_ = instance_counter_++;

  initResourcePaths();

  using namespace std::chrono;
  unsigned long long ticks =
      duration_cast<milliseconds>(steady_clock::now().time_since_epoch())
          .count();
  object_name_ = std::to_string(ticks);
}

void MeshVisual::initResourcePaths() {
  // set up resource paths
  std::string rviz_path = ros::package::getPath("rviz");

  // Adds folders with materials and script such that generic rviz materials
  // become available inside this plugin.
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      rviz_path + "/ogre_media/models", "FileSystem", ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      rviz_path + "/ogre_media/materials", "FileSystem", ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      rviz_path + "/ogre_media/materials/scripts", "FileSystem",
      ROS_PACKAGE_NAME);

  // Add paths exported to the "media_export" package.
  std::vector<std::string> media_paths;
  ros::package::getPlugins("media_export", "ogre_media_path", media_paths);
  std::string delim(":");
  for (std::vector<std::string>::iterator iter = media_paths.begin();
       iter != media_paths.end(); ++iter) {
    if (!iter->empty()) {
      std::string path;
      int pos1 = 0;
      int pos2 = iter->find(delim);
      while (pos2 != (int)std::string::npos) {
        path = iter->substr(pos1, pos2 - pos1);
        ROS_DEBUG("adding resource location: '%s'\n", path.c_str());
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
            path, "FileSystem", ROS_PACKAGE_NAME);
        pos1 = pos2 + 1;
        pos2 = iter->find(delim, pos2 + 1);
      }
      path = iter->substr(pos1, iter->size() - pos1);
      ROS_DEBUG("adding resource location: '%s'\n", path.c_str());
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          path, "FileSystem", ROS_PACKAGE_NAME);
    }
  }
}

// overloaded setMessage sets display mode
void MeshVisual::setMessage(const cgal_msgs::TriangleMeshStamped::ConstPtr &msg) {
  triangle_mesh_msg_ = msg->mesh;
}

void MeshVisual::setMessage(const cgal_msgs::ColoredMesh::ConstPtr &msg) {
  triangle_mesh_msg_ = msg->mesh;
  mesh_color_msg_ = msg->color;
  surface_colors_msg_ = msg->colors;
  visualize_color_ = true;
}

void MeshVisual::setMessage(const cgal_msgs::ProbabilisticMesh::ConstPtr &msg) {
  triangle_mesh_msg_ = msg->mesh;
  normals_msg_ = msg->normals;
  cov_vertices_msg_ = msg->cov_vertices;
  visualize_covariances_ = true;
}

MeshVisual::~MeshVisual() {}

void MeshVisual::update() {
  // check if there is a valid mesh
  // RVIZ crashes with empty meshes
  if(triangle_mesh_msg_.vertices.empty() ||
      triangle_mesh_msg_.triangles.empty()){
    ROS_WARN("Ignoring empty mesh.");
    return;
  }

  Ogre::ManualObject* ogre_object;

  if (scene_manager_->hasManualObject(object_name_)) {
    ogre_object = scene_manager_->getManualObject(object_name_);
  } else {
    ogre_object = scene_manager_->createManualObject(object_name_);
    frame_node_->attachObject(ogre_object);
  }
  ogre_object->clear();
  assert(ogre_object != nullptr);

  ogre_object->estimateVertexCount(3 * triangle_mesh_msg_.triangles.size());
  ogre_object->begin("BaseWhiteNoLighting",
                     Ogre::RenderOperation::OT_TRIANGLE_LIST);

  // Displaying Triangles in color defined by 3 vertex points
  uint vertex_count = 0;
  uint triangle_count = 0;
  for (auto triangle : triangle_mesh_msg_.triangles) {
    // every triangle needs distinct vertices such that colors do not interfere
    // possibility of color gradients over triangles is now removed
    for (uint i = 0; i < 3; i++) {
      ogre_object->position(
          triangle_mesh_msg_.vertices[triangle.vertex_indices[i]].x,
          triangle_mesh_msg_.vertices[triangle.vertex_indices[i]].y,
          triangle_mesh_msg_.vertices[triangle.vertex_indices[i]].z);
      if (visualize_color_) {
        if (surface_colors_msg_.size() != 0) {
          ogre_object->colour(Ogre::ColourValue(surface_colors_msg_[triangle_count].r,
                              surface_colors_msg_[triangle_count].g,
                              surface_colors_msg_[triangle_count].b,
                              surface_colors_msg_[triangle_count].a));
        }
        else if (mesh_color_msg_.r != 0 || mesh_color_msg_.g != 0 || mesh_color_msg_.b != 0) {
          ogre_object->colour(Ogre::ColourValue(mesh_color_msg_.r, mesh_color_msg_.g, 
                              mesh_color_msg_.b, mesh_color_msg_.a));
        }
        else {
          ogre_object->colour(std_surface_color_);
        }
      }
      else {
        ogre_object->colour(std_surface_color_);
      }
    }
    ogre_object->triangle(vertex_count, vertex_count + 1, vertex_count + 2);
    vertex_count += 3;
    triangle_count++;
  }
  ogre_object->end();

  // Displaying Edges
  ogre_object->begin("BaseWhiteNoLighting",
                     Ogre::RenderOperation::OT_LINE_LIST);
  for (auto triangle : triangle_mesh_msg_.triangles) {
    for (int i = 0; i < 3; ++i) {
      const auto& vrtx_a =
          triangle_mesh_msg_.vertices[triangle.vertex_indices[i]];
      ogre_object->position(vrtx_a.x, vrtx_a.y, vrtx_a.z);

      ogre_object->colour(std_edge_color_);

      const auto& vrtx_b =
          triangle_mesh_msg_.vertices[triangle.vertex_indices[(i + 1) % 3]];
      ogre_object->position(vrtx_b.x, vrtx_b.y, vrtx_b.z);
    }
  }

  // code to visualize covariances as lines. Disabled per default.
  if (visualize_covariances_) {
    for (size_t i = 0; i < triangle_mesh_msg_.vertices.size(); ++i) {
      Ogre::Vector3 mean;
      mean.x = triangle_mesh_msg_.vertices[i].x;
      mean.y = triangle_mesh_msg_.vertices[i].y;
      mean.z = triangle_mesh_msg_.vertices[i].z;

      Ogre::Vector3 stdev;
      stdev.x = sqrt(cov_vertices_msg_[i].x);
      stdev.y = sqrt(cov_vertices_msg_[i].y);
      stdev.z = sqrt(cov_vertices_msg_[i].z);

      Ogre::Vector3 start = mean - stdev, end = mean + stdev;

      // x cov
      ogre_object->position(start.x, mean.y, mean.z);
      ogre_object->position(end.x, mean.y, mean.z);

      // y cov
      ogre_object->position(mean.x, start.y, mean.z);
      ogre_object->position(mean.x, end.y, mean.z);

      // z cov
      ogre_object->position(mean.x, mean.y, start.z);
      ogre_object->position(mean.x, mean.y, end.z);
    }
  }

  Ogre::ManualObject::ManualObjectSection* section = ogre_object->end();
  const Ogre::MaterialPtr& mat = section->getMaterial();

  // set material properties for transparency
  mat->setSceneBlending(Ogre::SceneBlendType::SBT_TRANSPARENT_ALPHA);
  mat->setDepthWriteEnabled(false);

  // if backface culling is not desired, set culling mode to CULL_NONE
  mat->setCullingMode(backface_culling_ ? Ogre::CullingMode::CULL_CLOCKWISE
                                        : Ogre::CullingMode::CULL_NONE);
}

void MeshVisual::clear() {
  Ogre::ManualObject* ogre_object;
  if (scene_manager_->hasManualObject(object_name_)) {
    ogre_object = scene_manager_->getManualObject(object_name_);
    scene_manager_->destroyManualObject(ogre_object);
  }
}

void MeshVisual::setFramePosition(const Ogre::Vector3 &position) {
  frame_node_->setPosition(position);
}

void MeshVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
  frame_node_->setOrientation(orientation);
}

}  // namespace visualizations
}  // namespace cad_percept
