#include <unistd.h>
#include <Common/Util.h>
#include <Navio2/PWM.h>

#include <navio2_ros/pwm.hpp>

// Navio2 lib uses global namespace
PWM navio2_pwm;

namespace navio2_ros
{

PWM::PWM(rclcpp::Node *node, Pins pins)
{
  if (check_apm())
  {
    rclcpp::shutdown();
    return;
  }

  // Navio2 lib uses unsigned int, but node params does not have this value
  std::vector<int64_t> pins_param{pins.begin(), pins.end()};
  pins_param = node->declare_parameter("pins", pins_param);

  // check value when parsing param, put back to this->pins
  constexpr auto max_pin{13};
  for(auto pin: pins_param)
  {
    if(pin > max_pin || pin < 0)
      RCLCPP_WARN(node->get_logger(), "PWM does not have pin #%li, skipping", pin);
    else if(std::find(this->pins.begin(), this->pins.end(), pin) != this->pins.end())
      RCLCPP_WARN(node->get_logger(), "Pin #%li appears several times in the parameters, skipping", pin);
    else
      this->pins.push_back(pin);
  }

  initPWM();
}

PWM::~PWM()
{
  // stop pwms
  for(auto pin: pins)
    stop(pin);
  sleep(3);
}

void PWM::set_duty_cycle(Pin pin, float period)
{
  navio2_pwm.set_duty_cycle(pin, period/1000);
}

bool PWM::initPWM()
{
  // activate all outputs
  for(auto pin: pins)
  {
    if( !(navio2_pwm.init(pin)))
      return 1;
    navio2_pwm.set_period(pin, 50);
    if ( !(navio2_pwm.enable(pin)))
      return 1;
  }

  // arm
  for(auto pin: pins)
  {
    stop(pin);
    sleep(1);
  }
  return 0;
}

}
