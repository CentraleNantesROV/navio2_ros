#include <unistd.h>
#include <Common/Util.h>
#include <Navio2/PWM.h>

#include <navio2_ros/pwm.hpp>

namespace navio2_ros
{

PWM pwm;

PWM_Base::PWM_Base(rclcpp::NodeOptions options, Pins pins) : rclcpp::Node("pwm", options)
{
  if (check_apm())
  {
    rclcpp::shutdown();
    return;
  }

  // Navio2 lib uses unsigned int, but node params does not have this value
  std::vector<int64_t> pins_param;
  for(auto pin: pins)
    pins_param.push_back(pin);

  pins_param = declare_parameter("pins", pins_param);

  // check value when parsing param, put back to this->pins
  constexpr auto max_pin{13};
  for(auto pin: pins_param)
  {
    if(pin > max_pin || pin < 0)
      RCLCPP_WARN(get_logger(), "PWM does not have pin #%i, skipping", pin);
    else if(std::find(this->pins.begin(), this->pins.end(), pin) != this->pins.end())
      RCLCPP_WARN(get_logger(), "Pin #%i appears several times in the parameters, skipping", pin);
    else
      this->pins.push_back(pin);
  }

  initPWM();
}

PWM_Base::~PWM_Base()
{
  // stop pwms
  for(auto pin: pins)
    toRest(pin);
  sleep(3);
}

void PWM_Base::set_duty_cycle(Pin pin, float period)
{
  pwm.set_duty_cycle(pin, period/1000);
}

bool PWM_Base::initPWM()
{
  // activate all outputs
  for(auto pin: pins)
  {
    if( !(pwm.init(pin)))
      return 1;
    pwm.set_period(pin, 50);
    if ( !(pwm.enable(pin)))
      return 1;
  }

  // arm
  for(auto pin: pins)
  {
    toRest(pin);
    sleep(1);
  }
  return 0;
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(navio2_ros::PWM_Base)
