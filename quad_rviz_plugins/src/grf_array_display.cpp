#include "quad_rviz_plugins/grf_array_display.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <OgreColourValue.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include <QColor>

#include <pluginlib/class_list_macros.hpp>

#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/status_property.hpp>

namespace quad_rviz_plugins
{

namespace
{
Ogre::Vector3 toOgreVector(const geometry_msgs::msg::Point & point)
{
  return Ogre::Vector3(point.x, point.y, point.z);
}

Ogre::Vector3 toOgreVector(const geometry_msgs::msg::Vector3 & vector)
{
  return Ogre::Vector3(vector.x, vector.y, vector.z);
}
}  // namespace

GRFArrayDisplay::GRFArrayDisplay()
: arrows_node_(nullptr)
{
  active_color_property_ = new rviz_common::properties::ColorProperty(
    "Active Color", QColor(45, 31, 38),
    "Color used for GRFs with contact enabled.");
  inactive_color_property_ = new rviz_common::properties::ColorProperty(
    "Inactive Color", QColor(45, 31, 38),
    "Color used for GRFs with contact disabled.");
  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0, "Alpha applied to active GRF arrows.");
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  shaft_diameter_property_ = new rviz_common::properties::FloatProperty(
    "Shaft Diameter", 0.01, "Arrow shaft diameter in meters.");
  shaft_diameter_property_->setMin(0.0001);

  head_diameter_property_ = new rviz_common::properties::FloatProperty(
    "Head Diameter", 0.04, "Arrow head diameter in meters.");
  head_diameter_property_->setMin(0.0001);

  head_length_property_ = new rviz_common::properties::FloatProperty(
    "Head Length", 0.03, "Minimum arrow head length in meters.");
  head_length_property_->setMin(0.0001);

  force_scale_property_ = new rviz_common::properties::FloatProperty(
    "Force Scale", 0.002, "Scale factor from force magnitude to arrow length.");
  force_scale_property_->setMin(0.0);
}

GRFArrayDisplay::~GRFArrayDisplay()
{
  clearArrows();
  if (arrows_node_) {
    arrows_node_->removeAndDestroyAllChildren();
    scene_manager_->destroySceneNode(arrows_node_);
  }
}

void GRFArrayDisplay::onInitialize()
{
  MFDClass::onInitialize();
  arrows_node_ = scene_node_->createChildSceneNode();
}

void GRFArrayDisplay::reset()
{
  MFDClass::reset();
  clearArrows();
}

void GRFArrayDisplay::processMessage(quad_msgs::msg::GRFArray::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (msg->points.size() != msg->vectors.size() || msg->contact_states.size() != msg->vectors.size()) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Message",
      QString::fromStdString(
        "GRFArray field sizes differ: points=" + std::to_string(msg->points.size()) +
        ", vectors=" + std::to_string(msg->vectors.size()) +
        ", contact_states=" + std::to_string(msg->contact_states.size())));
    clearArrows();
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    setMissingTransformToFixedFrame(msg->header.frame_id);
    clearArrows();
    return;
  }

  setStatus(rviz_common::properties::StatusProperty::Ok, "Message", "OK");
  ensureArrowCount(msg->vectors.size());

  const QColor active_qcolor = active_color_property_->getColor();
  const QColor inactive_qcolor = inactive_color_property_->getColor();
  const Ogre::ColourValue active_color(
    active_qcolor.redF(), active_qcolor.greenF(), active_qcolor.blueF(), alpha_property_->getFloat());
  const Ogre::ColourValue inactive_color(
    inactive_qcolor.redF(), inactive_qcolor.greenF(), inactive_qcolor.blueF(), 0.0f);

  const float shaft_diameter = shaft_diameter_property_->getFloat();
  const float head_diameter = head_diameter_property_->getFloat();
  const float min_head_length = head_length_property_->getFloat();
  const float force_scale = force_scale_property_->getFloat();

  for (std::size_t i = 0; i < msg->vectors.size(); ++i) {
    Ogre::Vector3 base = orientation * toOgreVector(msg->points[i]) + position;
    Ogre::Vector3 direction = orientation * toOgreVector(msg->vectors[i]);
    const float total_length = direction.length() * force_scale;

    auto & arrow = arrows_[i];
    arrow->setPosition(base);

    if (total_length <= 1e-6f) {
      arrow->getSceneNode()->setVisible(false);
      continue;
    }

    direction.normalise();
    arrow->setDirection(direction);

    const float head_length = std::min(min_head_length, total_length);
    const float shaft_length = std::max(0.0f, total_length - head_length);
    arrow->set(shaft_length, shaft_diameter, head_length, head_diameter);
    arrow->setColor(msg->contact_states[i] ? active_color : inactive_color);
    arrow->getSceneNode()->setVisible(true);
  }

  context_->queueRender();
}

void GRFArrayDisplay::updateVisuals()
{
  context_->queueRender();
}

void GRFArrayDisplay::clearArrows()
{
  arrows_.clear();
}

void GRFArrayDisplay::ensureArrowCount(std::size_t count)
{
  while (arrows_.size() < count) {
    arrows_.push_back(std::make_unique<rviz_rendering::Arrow>(scene_manager_, arrows_node_));
  }
  while (arrows_.size() > count) {
    arrows_.pop_back();
  }
}

}  // namespace quad_rviz_plugins

PLUGINLIB_EXPORT_CLASS(
  quad_rviz_plugins::GRFArrayDisplay,
  rviz_common::Display)
