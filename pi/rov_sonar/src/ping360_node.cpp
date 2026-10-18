#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class DummyPublisher : public rclcpp::Node
{
public:
  DummyPublisher() : Node("ping360_node")
  {
    double publish_frequency_hz = this->declare_parameter<double>("publish_frequency_hz", 1.0);

    if (!std::isfinite(publish_frequency_hz) || publish_frequency_hz <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid publish_frequency_hz; falling back to 1.0 Hz");
      publish_frequency_hz = 1.0;
      this->set_parameter(rclcpp::Parameter("publish_frequency_hz", publish_frequency_hz));
    }

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("/sonar/ping360/scan", 10);
    const auto period = std::chrono::duration<double>(1.0 / publish_frequency_hz);
    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&DummyPublisher::publish_message, this));

    RCLCPP_INFO(get_logger(), "Ping360 dummy publisher started at %.1f Hz", publish_frequency_hz);
  }

private:
  void publish_message()
  {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = now();
    msg.header.frame_id = "ping360_link";
    msg.angle_min = -3.14159265;
    msg.angle_increment = 6.2831853 / 359.0;
    msg.angle_max = msg.angle_min + 359 * msg.angle_increment;
    msg.range_min = 0.3;
    msg.range_max = 50.0;
    msg.ranges.assign(360, 2.0);

    publisher_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyPublisher>());
  rclcpp::shutdown();
  return 0;
}
