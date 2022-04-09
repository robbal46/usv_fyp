#include <memory>
#include <array>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

using std::placeholders::_1;

class TwistAddCovariance : public rclcpp::Node
{
    public:
        TwistAddCovariance() : Node("twist_add_covariance"){
            twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10, std::bind(&TwistAddCovariance::twist_callback, this, _1));

            twist_cov_pub = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
                "/velocity", 10);

            this->declare_parameter<std::vector<double>>("covariance", std::vector<double>{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0});

            std::vector<double> cov_vec = this->get_parameter("covariance").as_double_array();
            std::copy_n(cov_vec.begin(), 36, covariance.begin());
        }

    private:
        void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg){

            auto twist_cov_msg = geometry_msgs::msg::TwistWithCovarianceStamped();

            twist_cov_msg.twist.twist.linear = msg->linear;
            twist_cov_msg.twist.twist.angular = msg->angular;

            twist_cov_msg.header.frame_id = "odom";
            twist_cov_msg.header.stamp = this->get_clock()->now();
            
            twist_cov_msg.twist.covariance = covariance;

            twist_cov_pub->publish(twist_cov_msg);
        }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;

        rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_cov_pub;

        std::array<double, 36UL> covariance;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistAddCovariance>());
    rclcpp::shutdown();
    return 0;
}