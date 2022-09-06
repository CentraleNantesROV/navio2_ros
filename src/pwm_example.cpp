#include <rclcpp/rclcpp.hpp>
#include <navio2_ros/pwm.hpp>
#include <navio2_ros/msg/pwm.hpp>

namespace navio2_ros
{

using namespace std::chrono_literals;

class PWMExample : public rclcpp::Node
{
public:
  explicit PWMExample(rclcpp::NodeOptions options) :
    Node("pwm", options),
    pwm(this, {0,1}) // activate pins 0 and 1 by default in this example
  {

    // stores last input and the corresponding time
    pwm_sub = create_subscription<msg::PWM>("pwm", 1, [&](msg::PWM::UniquePtr msg)
    {this->msg = *msg;last_pwm_time = now().seconds();});

    // prep message with default values
    msg = pwm.createMsg();

    // will reset all PWM output if no messages for 1 sec
    watchdog_timer = create_wall_timer(std::chrono::seconds(watchdog_period), [&](){watchDog();});

    // forward received pwm's to output @ 100 Hz
    pwm_timer = create_wall_timer(10ms, [&]()
    {
      pwm.apply(msg);
    });
  }

private:    

  PWM pwm;

  rclcpp::TimerBase::SharedPtr pwm_timer, watchdog_timer;

  rclcpp::Subscription<msg::PWM>::SharedPtr pwm_sub;
  msg::PWM msg;

  double last_pwm_time{};
  static constexpr auto watchdog_period{1};

  void watchDog()
  {
    if(now().seconds() - last_pwm_time > watchdog_period)
      pwm.stop();
  }
};

}

// boilerplate main
int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navio2_ros::PWMExample>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}

