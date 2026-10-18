#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

class DummyPublisher : public rclcpp::Node
{
public:
  DummyPublisher() : Node("bar30_node")
  {
    double publish_frequency_hz = this->declare_parameter<double>("publish_frequency_hz", 1.0);

    if (!std::isfinite(publish_frequency_hz) || publish_frequency_hz <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid publish_frequency_hz; falling back to 1.0 Hz");
      publish_frequency_hz = 1.0;
      this->set_parameter(rclcpp::Parameter("publish_frequency_hz", publish_frequency_hz));
    }

    publisher_ = create_publisher<sensor_msgs::msg::FluidPressure>("/sensors/pressure", 10);
    const auto period = std::chrono::duration<double>(1.0 / publish_frequency_hz);
    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&DummyPublisher::publish_message, this));

    RCLCPP_INFO(get_logger(), "BAR30 dummy publisher started at %.1f Hz", publish_frequency_hz);
  }

private:
  void publish_message()
  {
    sensor_msgs::msg::FluidPressure msg;
    msg.header.stamp = now();
    msg.header.frame_id = "bar30_link";
    msg.fluid_pressure = 101325.0;

    publisher_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyPublisher>());
  rclcpp::shutdown();
  return 0;
}
