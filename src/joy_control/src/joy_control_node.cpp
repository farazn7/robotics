#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class JoyControl : public rclcpp::Node {
public:
    JoyControl() : Node("joy_control"), drive_speed_(0.5) {
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyControl::joyCallback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        auto twist = geometry_msgs::msg::TwistStamped();

        twist.header.stamp = this->now();
        twist.header.frame_id = ""; 

        // Emergency stop 
        if (msg->buttons[12] == 1) {
            twist.twist.linear.x = 0.0;
            twist.twist.angular.z = 0.0;
            RCLCPP_WARN(this->get_logger(), "Emergency STOP!");
        } else {
            // Forward and backward 
            twist.twist.linear.x = msg->axes[3] * drive_speed_;
            if (msg->axes[3] > 0.1) {
                RCLCPP_INFO(this->get_logger(), "Forward: %.2f", twist.twist.linear.x);
            } else if (msg->axes[3] < -0.1) {
                RCLCPP_INFO(this->get_logger(), "Backward: %.2f", twist.twist.linear.x);
            }

            // Left and right
            twist.twist.angular.z = msg->axes[0] * drive_speed_;
            if (msg->axes[0] > 0.1) {
                RCLCPP_INFO(this->get_logger(), "Turn Left: %.2f", twist.twist.angular.z);
            } else if (msg->axes[0] < -0.1) {
                RCLCPP_INFO(this->get_logger(), "Turn Right: %.2f", twist.twist.angular.z);
            }

            // Speed control
            if (msg->buttons[6] == 1) {
                drive_speed_ = std::max(0.0, drive_speed_ - 0.1);
                RCLCPP_INFO(this->get_logger(), "Decrease speed: %.2f", drive_speed_);
            }
            if (msg->buttons[7] == 1) {
                drive_speed_ = std::min(1.5, drive_speed_ + 0.1);
                RCLCPP_INFO(this->get_logger(), "Increase speed: %.2f", drive_speed_);
            }
        }

        cmd_vel_publisher_->publish(twist);
    }

    double drive_speed_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
