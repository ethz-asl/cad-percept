#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <tf/transform_listener.h>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include <cgal_visualizations/probabilistic_mesh_display.h>

namespace cad_percept {

namespace visualizations {

void ProbabilisticMeshDisplay::onInitialize() { MFDClass::onInitialize(); }

ProbabilisticMeshDisplay::~ProbabilisticMeshDisplay() {}

void ProbabilisticMeshDisplay::reset() {
  MFDClass::reset();
  visual_.reset();
}

void ProbabilisticMeshDisplay::processMessage(
    const cgal_msgs::ProbabilisticMesh::ConstPtr& msg) {
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
      msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  if (visual_ == nullptr) {
    visual_.reset(
        new ProbabilisticMeshVisual(context_->getSceneManager(), scene_node_));
  }

  // Now set or update the contents of the chosen visual.
  visual_->setMessage(msg);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
}

}  // namespace visualizations
}  // namespace cad_percept

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cad_percept::visualizations::ProbabilisticMeshDisplay,
                       rviz::Display)
