#ifndef NAVIO2_ROS_PWM_H
#define NAVIO2_ROS_PWM_H

#include <rclcpp/node.hpp>

namespace navio2_ros
{

using Pin = unsigned int;
using Pins = std::vector<Pin>;

class PWM_Base : public rclcpp::Node
{
public:
  PWM_Base(rclcpp::NodeOptions options, Pins pins = {});
  ~PWM_Base();

protected:

  Pins pins;
  static constexpr float rest_duty_cycle{1500.f};
  void set_duty_cycle(Pin pin, float period);

  inline void toRest(Pin pin)
  {
    set_duty_cycle(pin, rest_duty_cycle);
  }

  bool initPWM();
};

}


#endif
