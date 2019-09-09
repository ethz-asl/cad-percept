#include <OGRE/OgreSceneNode.h>
#include <cgal_visualizations/mesh_display.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>

namespace cad_percept {
namespace visualizations {

template <typename T>
MeshDisplay<T>::MeshDisplay() {}

template <typename T>
void MeshDisplay<T>::onInitialize() {
  initProperties();
  rviz::MessageFilterDisplay<T>::MFDClass::onInitialize();
  std::cout << "onInitialize" << std::endl;
}

template <typename T>
MeshDisplay<T>::~MeshDisplay() {}

template <typename T>
void MeshDisplay<T>::initProperties() {
  properties_.BackfaceCulling = new rviz::BoolProperty(
      "Backface Culling", false,
      "If backface culling is active, a surface is only visible from the front "
      "face defined by the normal",
      this, SLOT(backfaceCullingPropertyChanged()));

  properties_.SurfaceColor =
      new rviz::ColorProperty("Surface Color", QColor(0, 0, 255), "Color of surfaces.", this,
                              SLOT(appearencePropertyChanged()));

  properties_.EdgeColor = new rviz::ColorProperty(
      "Edge Color", QColor(0, 0, 0), "Color of surfaces.", this, SLOT(appearencePropertyChanged()));

  properties_.Alpha = new rviz::FloatProperty("Alpha", 0.8, "Transparency", this,
                                              SLOT(appearencePropertyChanged()));
  properties_.Alpha->setMax(1.0);
  properties_.Alpha->setMin(0.0);
}

template <typename T>
void MeshDisplay<T>::reset() {
  if (visual_ != nullptr) {
    visual_->clear();
  }
  visual_.reset();
}

template <typename T>
void MeshDisplay<T>::backfaceCullingPropertyChanged() {
  if (visual_ != nullptr) {
    visual_->setBackFaceCulling(properties_.BackfaceCulling->getBool());
    visual_->update();
  }
}

template <typename T>
void MeshDisplay<T>::appearencePropertyChanged() {
  if (visual_ != nullptr) {
    visual_->setAppearance(properties_.EdgeColor->getOgreColor(),
                           properties_.SurfaceColor->getOgreColor(), properties_.Alpha->getFloat());
    visual_->update();
  }
}

template <typename T>
void MeshDisplay<T>::updateVisualProperties() {
  // call all propertychanged functions to update underlying visual.
  backfaceCullingPropertyChanged();
  appearencePropertyChanged();
}

template <typename T>
void MeshDisplay<T>::processMessage(const typename T::ConstPtr &msg) {
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!rviz::MessageFilterDisplay<T>::context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
              qPrintable(rviz::MessageFilterDisplay<T>::fixed_frame_));
    return;
  }

  if (visual_ == nullptr) {
    visual_.reset(new MeshVisual(rviz::MessageFilterDisplay<T>::context_->getSceneManager(),
                                 rviz::MessageFilterDisplay<T>::scene_node_));
  }

  // Now set or update the contents of the chosen visual.
  visual_->setMessage(msg);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);

  // update visuals.
  updateVisualProperties();

  visual_->update();
}

}  // namespace visualizations
}  // namespace cad_percept

/*
 * Export Pluginlib interfaces.
 */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cad_percept::visualizations::TriangleMeshDisplay, rviz::Display)
PLUGINLIB_EXPORT_CLASS(cad_percept::visualizations::ColoredMeshDisplay, rviz::Display)
PLUGINLIB_EXPORT_CLASS(cad_percept::visualizations::ProbabilisticMeshDisplay, rviz::Display)
