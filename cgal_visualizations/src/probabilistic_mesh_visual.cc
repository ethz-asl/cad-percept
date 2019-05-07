#include <cgal_visualizations/probabilistic_mesh_visual.h>

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

unsigned int ProbabilisticMeshVisual::instance_counter_ = 0;

ProbabilisticMeshVisual::ProbabilisticMeshVisual(
    Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
    : visualize_covariances_(false),
      backface_culling_(false),
      edge_color_(0.0, 0.0, 0.0, 0.8),
      surface_color_(0.0, 0.0, 1.0, 0.8) {
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

void ProbabilisticMeshVisual::initResourcePaths() {
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

void ProbabilisticMeshVisual::setMessage(
    const cgal_msgs::ProbabilisticMesh::ConstPtr& msg) {
  msg_ = msg;
}

ProbabilisticMeshVisual::~ProbabilisticMeshVisual() {}

void ProbabilisticMeshVisual::update() {
  Ogre::ManualObject* ogre_object;

  if (scene_manager_->hasManualObject(object_name_)) {
    ogre_object = scene_manager_->getManualObject(object_name_);
  } else {
    ogre_object = scene_manager_->createManualObject(object_name_);
    frame_node_->attachObject(ogre_object);
  }
  ogre_object->clear();
  assert(ogre_object != nullptr);

  ogre_object->estimateVertexCount(msg_->cov_vertices.size());
  ogre_object->begin("BaseWhiteNoLighting",
                     Ogre::RenderOperation::OT_TRIANGLE_LIST);

  // Displaying Vertices
  for (auto vrtx : msg_->mesh.vertices) {
    ogre_object->position(vrtx.x, vrtx.y, vrtx.z);

    ogre_object->colour(surface_color_);
  }

  // Displaying Triangles
  for (auto triangle : msg_->mesh.triangles) {
    ogre_object->triangle(triangle.vertex_indices[2],
                          triangle.vertex_indices[1],
                          triangle.vertex_indices[0]);
  }
  ogre_object->end();

  // Displaying Edges
  ogre_object->begin("BaseWhiteNoLighting",
                     Ogre::RenderOperation::OT_LINE_LIST);
  for (auto triangle : msg_->mesh.triangles) {
    for (int i = 0; i < 3; ++i) {
      const auto& vrtx_a = msg_->mesh.vertices[triangle.vertex_indices[i]];
      ogre_object->position(vrtx_a.x, vrtx_a.y, vrtx_a.z);

      ogre_object->colour(edge_color_);

      const auto& vrtx_b =
          msg_->mesh.vertices[triangle.vertex_indices[(i + 1) % 3]];
      ogre_object->position(vrtx_b.x, vrtx_b.y, vrtx_b.z);
    }
  }

  // code to visualize covariances as lines. Disabled per default.
  if (visualize_covariances_) {
    for (size_t i = 0; i < msg_->mesh.vertices.size(); ++i) {
      Ogre::Vector3 mean;
      mean.x = msg_->mesh.vertices[i].x;
      mean.y = msg_->mesh.vertices[i].y;
      mean.z = msg_->mesh.vertices[i].z;

      Ogre::Vector3 stdev;
      stdev.x = sqrt(msg_->cov_vertices[i].x);
      stdev.y = sqrt(msg_->cov_vertices[i].y);
      stdev.z = sqrt(msg_->cov_vertices[i].z);

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

void ProbabilisticMeshVisual::clear() {
  Ogre::ManualObject* ogre_object;
  if (scene_manager_->hasManualObject(object_name_)) {
    ogre_object = scene_manager_->getManualObject(object_name_);
    scene_manager_->destroyManualObject(ogre_object);
  }
}

void ProbabilisticMeshVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void ProbabilisticMeshVisual::setFrameOrientation(
    const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

}  // namespace visualization
}  // namespace cad_percept
