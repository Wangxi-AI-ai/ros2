#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <random>

class DataPublisher : public rclcpp::Node
{
public:
    DataPublisher() : Node("data_publisher"), gen(rd()), dist(0.0, 0.2)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("raw_data", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
            std::bind(&DataPublisher::publish_data, this));
        RCLCPP_INFO(this->get_logger(), "Data Publisher started");
    }

private:
    void publish_data()
    {
        static double t = 0.0;
        double freq = 1.0; // 1 Hz
        double signal = std::sin(2 * M_PI * freq * t);
        double noisy_signal = signal + dist(gen);
        t += 0.05;

        std_msgs::msg::Float64 msg;
        msg.data = noisy_signal;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::random_device rd;
    std::mt19937 gen;
    std::normal_distribution<double> dist;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataPublisher>());
    rclcpp::shutdown();
    return 0;
}
