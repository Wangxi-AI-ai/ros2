#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>

class SignalProcessor : public rclcpp::Node
{
public:
    SignalProcessor() : Node("signal_processor")
    {
        sine_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "sine_signal", 10, std::bind(&SignalProcessor::sine_callback, this, std::placeholders::_1));

        square_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "square_signal", 10, std::bind(&SignalProcessor::square_callback, this, std::placeholders::_1));

        output_pub_ = this->create_publisher<std_msgs::msg::Float64>("processed_signal", 10);

        RCLCPP_INFO(this->get_logger(), "Signal processor node started!");
    }

private:
    void sine_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        last_sine_ = msg->data;
        process_and_publish();
    }

    void square_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        last_square_ = msg->data;
        process_and_publish();
    }

    void process_and_publish()
    {
        if (!std::isnan(last_sine_) && !std::isnan(last_square_))
        {
            std_msgs::msg::Float64 output_msg;
            if ((last_sine_ >= 0 && last_square_ >= 0) || (last_sine_ < 0 && last_square_ < 0))
                output_msg.data = last_sine_;
            else
                output_msg.data = 0.0;

            output_pub_->publish(output_msg);

            // 每隔一定次数打印一次
            if (count_ % 100 == 0)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Input: sine=%.3f, square=%.1f -> Output=%.3f",
                            last_sine_, last_square_, output_msg.data);
            }
            count_++;
        }
    }

    double last_sine_ = NAN;
    double last_square_ = NAN;
    int count_ = 0;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sine_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr square_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr output_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalProcessor>());
    rclcpp::shutdown();
    return 0;
}