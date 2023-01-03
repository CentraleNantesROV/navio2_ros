
#include <Common/Util.h>
#include <Navio2/Led_Navio2.h>

#include <rclcpp/node.hpp>
#include <navio2_ros/msg/navio_led.hpp>

namespace navio2_ros
{

class LEDNode : public rclcpp::Node
{
public:
  explicit LEDNode(rclcpp::NodeOptions options) : rclcpp::Node("led", options)
  {
    if (check_apm())
    {
      rclcpp::shutdown();
      return;
    }

    led.initialize();

    led_sub = create_subscription<msg::NavioLED>("led", 1, [&](msg::NavioLED::UniquePtr msg)
    {led_callback(msg->color);});
  }

protected:

  Led_Navio2 led;

  rclcpp::Subscription<msg::NavioLED>::SharedPtr led_sub;

  void led_callback(uint8_t color)
  {
    static const std::map<uint8_t, Colors> colors{
      {msg::NavioLED::RED, Colors::Red},
      {msg::NavioLED::BLUE, Colors::Blue},
      {msg::NavioLED::CYAN, Colors::Cyan},
      {msg::NavioLED::BLACK, Colors::Black},
      {msg::NavioLED::GREEN, Colors::Green},
      {msg::NavioLED::YELLOW, Colors::Yellow},
      {msg::NavioLED::MAGENTA, Colors::Magenta},
      {msg::NavioLED::WHITE, Colors::White}
    };

    led.setColor(colors.at(color));
  }

};
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navio2_ros::LEDNode)
