#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class LowPassFilterNode : public rclcpp::Node
{
public:
    LowPassFilterNode() : Node("lowpass_filter"), alpha_(0.2), y_prev_(0.0)
    {
        sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "raw_data", 10, std::bind(&LowPassFilterNode::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<std_msgs::msg::Float64>("lowpass_filtered", 10);
        RCLCPP_INFO(this->get_logger(), "Low-pass Filter started");
    }

private:
    void callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double x = msg->data;
        double y = alpha_ * x + (1.0 - alpha_) * y_prev_;
        y_prev_ = y;

        std_msgs::msg::Float64 out_msg;
        out_msg.data = y;
        pub_->publish(out_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
    double alpha_;
    double y_prev_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LowPassFilterNode>());
    rclcpp::shutdown();
    return 0;
}
