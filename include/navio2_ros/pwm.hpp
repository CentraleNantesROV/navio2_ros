#ifndef NAVIO2_ROS_PWM_H
#define NAVIO2_ROS_PWM_H

#include <rclcpp/node.hpp>
#include <navio2_ros/msg/pwm.hpp>

namespace navio2_ros
{

using Pin = unsigned int;
using Pins = std::vector<Pin>;

class PWM
{
public:
  PWM(rclcpp::Node* node, Pins pins = {});
  ~PWM();
  void set_duty_cycle(Pin pin, float period);

  inline void stop(Pin pin)
  {
    set_duty_cycle(pin, rest_duty_cycle);
  }
  inline void stop()
  {
    for(auto pin: pins)
      stop(pin);
  }
  inline void apply(const msg::PWM &pwm)
  {
    for(size_t i = 0; i < pwm.pin.size(); ++i)
      set_duty_cycle(pwm.pin[i], pwm.duty_cycle[i]);
  }
  inline msg::PWM createMsg() const
  {
    msg::PWM msg;
    std::copy(pins.begin(), pins.end(), std::back_inserter(msg.pin));
    msg.duty_cycle.resize(msg.pin.size(), rest_duty_cycle);
    return msg;
  }

protected:

  Pins pins;
  static constexpr float rest_duty_cycle{1500.f};

  bool initPWM();
};

}


#endif
