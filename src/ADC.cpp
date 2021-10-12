#include <unistd.h>
#include <Common/Util.h>
#include <Navio2/ADC_Navio2.h>

#include <rclcpp/node.hpp>
#include <navio2_ros/msg/adc.hpp>

#define READ_FAILED -1

namespace navio2_ros
{

using namespace std::chrono_literals;

class ADCReader : public rclcpp::Node
{
public:
  ADCReader(rclcpp::NodeOptions options) : rclcpp::Node("adc", options)
  {
    if (check_apm())
    {
      rclcpp::shutdown();
      return;
    }

    adc = std::make_unique<ADC_Navio2>();
    adc->initialize();

    adc_pub = create_publisher<msg::ADC>("adc", rclcpp::SensorDataQoS());
    publish_timer = create_wall_timer(1s, [&](){publish();});
  }

protected:

  std::unique_ptr <ADC> adc;

  rclcpp::TimerBase::SharedPtr publish_timer;
  rclcpp::Publisher<msg::ADC>::SharedPtr adc_pub;
  msg::ADC msg;

  void publish()
  {
    for (int channel = 0; channel < adc->get_channel_count(); channel++)
    {
      const float value = adc->read(channel);
      if(value == READ_FAILED)
        return;
      msg.channel[channel] = value;
    }
    adc_pub->publish(msg);
  }
};
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navio2_ros::ADCReader)
