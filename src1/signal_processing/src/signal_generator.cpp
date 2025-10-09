#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>

class SignalGenerator : public rclcpp::Node
{
public:
  SignalGenerator()
  : Node("signal_generator"), time_(0.0)
  {
    // 创建两个发布者
    sine_pub_ = this->create_publisher<std_msgs::msg::Float64>("sine_signal", 10);
    square_pub_ = this->create_publisher<std_msgs::msg::Float64>("square_signal", 10);

    // 设置定时器：1000Hz（1ms一次），更平滑的曲线
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&SignalGenerator::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Signal generator node started!");
  }

private:
  void timer_callback()
  {
    // 每次递增时间
    time_ += 0.001;  // 1ms步进

    // 生成信号
    std_msgs::msg::Float64 sine_msg;
    std_msgs::msg::Float64 square_msg;

    // 正弦信号：10Hz
    sine_msg.data = std::sin(2 * M_PI * 10.0 * time_);

    // 方波信号：1Hz
    square_msg.data = (std::fmod(time_, 1.0) < 0.5) ? 1.0 : -1.0;

    // 发布信号
    sine_pub_->publish(sine_msg);
    square_pub_->publish(square_msg);

    // 打印
    RCLCPP_INFO(this->get_logger(), "Sine: %.3f, Square: %.1f", sine_msg.data, square_msg.data);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sine_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr square_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SignalGenerator>());
  rclcpp::shutdown();
  return 0;
}
