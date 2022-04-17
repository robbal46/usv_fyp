#include <memory>
#include <array>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using std::placeholders::_1;

class DeadReckoning : public rclcpp::Node
{
    public:
        DeadReckoning() : Node("twist_add_covariance"){
            twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10, std::bind(&DeadReckoning::twist_callback, this, _1));

            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odometry/filtered", 10, std::bind(&DeadReckoning::odom_cb, this, _1));

            odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(
                "/odometry/dead_reckoning", 10);

            prev =this->get_clock()->now();

            this->declare_parameter<std::vector<double>>("covariance", std::vector<double>{
                0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0});
            std::vector<double> cov_vec = this->get_parameter("covariance").as_double_array();
            std::copy_n(cov_vec.begin(), 36, covariance.begin());
        }

    private:
        void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg){

            auto odom_msg = nav_msgs::msg::Odometry();

            odom_msg.twist.twist.linear = msg->linear;
            odom_msg.twist.twist.angular = msg->angular;
            odom_msg.twist.covariance = covariance;

            rclcpp::Time now = this->get_clock()->now();
            double dt = (now - prev).seconds();

            x += odom_msg.twist.twist.linear.x * cos(th) * dt;
            y += odom_msg.twist.twist.linear.x * sin(th) * dt;

            odom_msg.pose.pose.position.x = x;
            odom_msg.pose.pose.position.y = y;
            odom_msg.pose.covariance = covariance;

            odom_msg.header.frame_id = "odom";
            odom_msg.header.stamp = now;

            odom_pub->publish(odom_msg);

            prev = now;
        }

        // Get yaw from filtered odom msg (cyclical, but fine?)
        void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg){
            tf2::Quaternion q(msg->pose.pose.orientation.x,
                              msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z,
                              msg->pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            th = yaw;
        }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

        double x = 0;
        double y = 0;
        double th = 0;
        rclcpp::Time prev;

        std::array<double, 36UL> covariance;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeadReckoning>());
    rclcpp::shutdown();
    return 0;
}