#ifndef QUAD_RVIZ_PLUGINS__GRF_ARRAY_DISPLAY_HPP_
#define QUAD_RVIZ_PLUGINS__GRF_ARRAY_DISPLAY_HPP_

#include <memory>
#include <vector>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <quad_msgs/msg/grf_array.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_rendering/objects/arrow.hpp>

namespace quad_rviz_plugins
{

class GRFArrayDisplay : public rviz_common::MessageFilterDisplay<quad_msgs::msg::GRFArray>
{
public:
  GRFArrayDisplay();
  ~GRFArrayDisplay() override;

  void onInitialize() override;
  void reset() override;

protected:
  void processMessage(quad_msgs::msg::GRFArray::ConstSharedPtr msg) override;

private:
  void updateVisuals();
  void clearArrows();
  void ensureArrowCount(std::size_t count);

  Ogre::SceneNode * arrows_node_;
  std::vector<std::unique_ptr<rviz_rendering::Arrow>> arrows_;

  rviz_common::properties::ColorProperty * active_color_property_;
  rviz_common::properties::ColorProperty * inactive_color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::FloatProperty * shaft_diameter_property_;
  rviz_common::properties::FloatProperty * head_diameter_property_;
  rviz_common::properties::FloatProperty * head_length_property_;
  rviz_common::properties::FloatProperty * force_scale_property_;
};

}  // namespace quad_rviz_plugins

#endif  // QUAD_RVIZ_PLUGINS__GRF_ARRAY_DISPLAY_HPP_
