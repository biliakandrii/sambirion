#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <string>

class JointStatePublisherNode : public rclcpp::Node
{
public:
   JointStatePublisherNode()
    : Node("joint_state_publisher_node")
    {
        // Enable simulation time
        // this->declare_parameter("use_sim_time", true);

        // Publisher for joint states
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Subscriber for cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&JointStatePublisherNode::cmdVelCallback, this, std::placeholders::_1));

        // Initialize wheel joint names
        joint_names_ = {
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint"
        };
        wheel_angles_ = std::vector<double>(4, 0.0);

        last_time_ = this->now();

        // Timer runs at 20 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&JointStatePublisherNode::publishJointStates, this)
        );

        RCLCPP_INFO(this->get_logger(), "JointStatePublisherNode initialized with sim time.");
    }


private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // Simple mecanum wheel approximation: all wheels rotate proportionally to linear x
        // You can improve this with full mecanum kinematics if needed
        double v = msg->linear.x;      // forward velocity
        double w = msg->angular.z;     // rotation velocity around z
        double wheel_radius = 0.1;     // meters
        double half_width = 0.1625;    // half of chassis width (from SDF)
        double half_length = 0.2;      // half of chassis length (from SDF)

        // Wheel angular velocities (rad/s)
        double wl = (v - w * (half_width + half_length)) / wheel_radius;
        double wr = (v + w * (half_width + half_length)) / wheel_radius;

        // Update accumulated angles
        wheel_angles_[0] += wl * dt; // front left
        wheel_angles_[1] += wr * dt; // front right
        wheel_angles_[2] += wl * dt; // rear left
        wheel_angles_[3] += wr * dt; // rear right

        // Publish joint states
        sensor_msgs::msg::JointState js;
        js.header.stamp = current_time;
        js.name = joint_names_;
        js.position = wheel_angles_;

        joint_state_pub_->publish(js);
    }
    // Separate function
    void publishJointStates()
    {
        auto current_time = this->now();
        sensor_msgs::msg::JointState js;
        js.header.stamp = current_time;
        js.name = joint_names_;
        js.position = wheel_angles_; // whatever current angles are
        joint_state_pub_->publish(js);
    }
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;  // <--- Add this

    std::vector<std::string> joint_names_;
    std::vector<double> wheel_angles_;
    rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisherNode>());
    rclcpp::shutdown();
    return 0;
}
