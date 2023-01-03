
#include <Common/Util.h>
#include <Common/MS5611.h>
#include <unistd.h>

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

namespace navio2_ros
{

using sensor_msgs::msg::Temperature;
using sensor_msgs::msg::FluidPressure;
using namespace std::chrono_literals;

class Barometer : public rclcpp::Node
{
public:
  explicit Barometer(rclcpp::NodeOptions options) : rclcpp::Node("barometer", options)
  {
    if (check_apm())
    {
      rclcpp::shutdown();
      return;
    }
    barometer.initialize();

    pressure_msg.header.frame_id
        = temperature_msg.header.frame_id
          = declare_parameter<std::string>("barometer_frame", "baro");

    pressure_msg.variance = temperature_msg.variance = 0;

    temperature_pub = create_publisher<Temperature>("temperature", rclcpp::SensorDataQoS());
    pressure_pub = create_publisher<FluidPressure>("pressure", rclcpp::SensorDataQoS());

    publish_timer = create_wall_timer(1s, [&](){publish();});
  }

protected:

  MS5611 barometer;

  rclcpp::Publisher<Temperature>::SharedPtr temperature_pub;
  rclcpp::Publisher<FluidPressure>::SharedPtr pressure_pub;
  Temperature temperature_msg;
  FluidPressure pressure_msg;

  rclcpp::TimerBase::SharedPtr publish_timer;

  void publish()
  {
    barometer.refreshPressure();
    usleep(10000); // Waiting for pressure data ready
    barometer.readPressure();

    barometer.refreshTemperature();
    usleep(10000); // Waiting for temperature data ready
    barometer.readTemperature();

    barometer.calculatePressureAndTemperature();

    temperature_msg.temperature = barometer.getTemperature();

    // Navio2 outputs millibars, FluidPressure wants Pascals
    pressure_msg.fluid_pressure = barometer.getPressure()/100;

    temperature_msg.header.stamp = pressure_msg.header.stamp = now();

    temperature_pub->publish(temperature_msg);
    pressure_pub->publish(pressure_msg);
  }
};
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navio2_ros::Barometer)
