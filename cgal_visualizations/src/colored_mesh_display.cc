#include <cgal_visualizations/colored_mesh_display.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>

namespace cad_percept {

namespace visualizations {

void ColoredMeshDisplay::onInitialize() {
  MFDClass::onInitialize();
  initProperties();
}

ColoredMeshDisplay::~ColoredMeshDisplay() {}

void ColoredMeshDisplay::reset() {
  MFDClass::reset();

  if (visual_ != nullptr) {
    visual_->clear();
  }
  visual_.reset();
}

void ColoredMeshDisplay::initProperties() {
  properties_.BackfaceCulling = new rviz::BoolProperty(
      "Backface Culling", false,
      "If backface culling is active, a surface is only visible from the front "
      "face defined by the normal",
      this, SLOT(backfaceCullingPropertyChanged()));

  properties_.EdgeColor = new rviz::ColorProperty(
      "Edge Color", QColor(0, 0, 0), "Color of surfaces.", this, SLOT(appearencePropertyChanged()));

  properties_.Alpha = new rviz::FloatProperty("Alpha", 0.8, "Transparency", this,
                                              SLOT(appearencePropertyChanged()));
  properties_.Alpha->setMax(1.0);
  properties_.Alpha->setMin(0.0);
}

void ColoredMeshDisplay::backfaceCullingPropertyChanged() {
  if (visual_ != nullptr) {
    visual_->setBackFaceCulling(properties_.BackfaceCulling->getBool());
    visual_->update();
  }
}

void ColoredMeshDisplay::appearencePropertyChanged() {
  if (visual_ != nullptr) {
    visual_->setAppearance(properties_.EdgeColor->getOgreColor(),
                           Ogre::ColourValue(0.0, 0.0, 1.0, 0.8), properties_.Alpha->getFloat());
    visual_->update();
  }
}

void ColoredMeshDisplay::processMessage(const cgal_msgs::ColoredMesh::ConstPtr &msg) {
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position,
                                                 orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
              qPrintable(fixed_frame_));
    return;
  }

  if (visual_ == nullptr) {
    visual_.reset(new MeshVisual(context_->getSceneManager(), scene_node_));
  }

  // Now set or update the contents of the chosen visual.
  visual_->setMessage(msg);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
  visual_->update();
}

}  // namespace visualizations
}  // namespace cad_percept

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cad_percept::visualizations::ColoredMeshDisplay, rviz::Display)
