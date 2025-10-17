#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <deque>
#include <algorithm>

class MedianFilterNode : public rclcpp::Node
{
public:
    MedianFilterNode() : Node("median_filter"), window_size_(5)
    {
        sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "raw_data", 10, std::bind(&MedianFilterNode::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<std_msgs::msg::Float64>("median_filtered", 10);
        RCLCPP_INFO(this->get_logger(), "Median Filter started");
    }

private:
    void callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        window_.push_back(msg->data);
        if (window_.size() > window_size_)
            window_.pop_front();

        std::vector<double> temp(window_.begin(), window_.end());
        std::sort(temp.begin(), temp.end());
        double median = temp[temp.size() / 2];

        std_msgs::msg::Float64 out_msg;
        out_msg.data = median;
        pub_->publish(out_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
    std::deque<double> window_;
    size_t window_size_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MedianFilterNode>());
    rclcpp::shutdown();
    return 0;
}
