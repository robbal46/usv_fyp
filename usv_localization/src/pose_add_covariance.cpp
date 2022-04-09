/*
pose_add_covariance

Takes a geometry_msgs/PoseStamped message from orb_slam2_ros and converts
this to a geometry_msgs/PoseWithCovarianceStamped message to feed into 
robot_localization EKF node by adding a covariance matrix

Covariance matrix is 36 element (6x6) double array that can be specified as a parameter
*/

#include <memory>
#include <array>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using std::placeholders::_1;

class PoseAddCovariance : public rclcpp::Node
{
    public:
        PoseAddCovariance() : Node("pose_add_covariance"){
            pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/orb_slam2_mono_node/pose", 10, std::bind(&PoseAddCovariance::pose_callback, this, _1));

            pose_cov_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/camera/pose", 10);

            this->declare_parameter<std::vector<double>>("covariance", std::vector<double>{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0});

            std::vector<double> cov_vec = this->get_parameter("covariance").as_double_array();
            std::copy_n(cov_vec.begin(), 36, covariance.begin());
        }

    private:
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){

            auto pose_cov_msg = geometry_msgs::msg::PoseWithCovarianceStamped();

            pose_cov_msg.pose.pose = msg->pose;
            pose_cov_msg.header = msg->header;
            
            pose_cov_msg.pose.covariance = covariance;

            pose_cov_pub->publish(pose_cov_msg);
        }

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;

        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub;

        std::array<double, 36UL> covariance;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseAddCovariance>());
    rclcpp::shutdown();
    return 0;
}

