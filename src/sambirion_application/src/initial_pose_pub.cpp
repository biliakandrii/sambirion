#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

using namespace std::chrono_literals;

class InitialPosePublisher : public rclcpp::Node
{
public:
    InitialPosePublisher()
    : Node("initial_pose_pub")
    {
        // Declare parameters
        this->declare_parameter<double>("initial_x", 0.0);
        this->declare_parameter<double>("initial_y", 0.0);
        this->declare_parameter<double>("initial_yaw", 0.0);

        // Create publisher
        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

        // Wait a short time to allow other nodes to start (map_server, AMCL)
        timer_ = this->create_wall_timer(
            500ms, std::bind(&InitialPosePublisher::publish_initial_pose, this));
    }

private:
    void publish_initial_pose()
    {
        double x = this->get_parameter("initial_x").as_double();
        double y = this->get_parameter("initial_y").as_double();
        double yaw = this->get_parameter("initial_yaw").as_double();

        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = 0.0;
        msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
        msg.pose.pose.orientation.w = std::cos(yaw / 2.0);

        // Small covariance
        msg.pose.covariance = {
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 0.25, 0, 0, 0,
            0, 0, 0, 0.0685, 0, 0,
            0, 0, 0, 0, 0.0685, 0,
            0, 0, 0, 0, 0, 0.0685
        };

        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published initial pose: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);

        // shutdown after publishing once
        rclcpp::shutdown();
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
