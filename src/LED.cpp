
#include <Common/Util.h>
#include <Navio2/Led_Navio2.h>

#include <rclcpp/node.hpp>
#include <navio2_ros/msg/led.hpp>

namespace navio2_ros
{

class LED : public rclcpp::Node
{
public:
  explicit LED(rclcpp::NodeOptions options) : rclcpp::Node("led", options)
  {
    if (check_apm())
    {
      rclcpp::shutdown();
      return;
    }

    led.initialize();

    led_sub = create_subscription<msg::LED>("led", 1, [&](msg::LED::UniquePtr msg)
    {led_callback(msg->color);});
  }

protected:

  Led_Navio2 led;

  rclcpp::Subscription<msg::LED>::SharedPtr led_sub;

  void led_callback(uint8_t color)
  {
    static const std::map<uint8_t, Colors> colors{
      {msg::LED::RED, Colors::Red},
      {msg::LED::BLUE, Colors::Blue},
      {msg::LED::CYAN, Colors::Cyan},
      {msg::LED::BLACK, Colors::Black},
      {msg::LED::GREEN, Colors::Green},
      {msg::LED::YELLOW, Colors::Yellow},
      {msg::LED::MAGENTA, Colors::Magenta},
      {msg::LED::WHITE, Colors::White}
    };

    led.setColor(colors.at(color));
  }

};
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navio2_ros::LED)
