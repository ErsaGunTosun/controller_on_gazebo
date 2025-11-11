#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"


class JoyController: public rclcpp::Node
{
	public:
		JoyController():Node("joy_controller"){
			this->declare_parameter<double>("scale_linear", 0.5);
      this->declare_parameter<double>("scale_angular", 1.0);

      this->get_parameter("scale_linear", scale_linear_);
      this->get_parameter("scale_angular", scale_angular_);

      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

			subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, 
					std::bind(&JoyController::joy_callback, this, std::placeholders::_1));
		}
	private:
		void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      auto twist_msg = geometry_msgs::msg::Twist();

      twist_msg.linear.x = msg->axes[1] * scale_linear_;

      twist_msg.angular.z = msg->axes[0] * scale_angular_;
      
      publisher_->publish(twist_msg);
    }
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
		double scale_linear_;
		double scale_angular_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc,argv);
  
  auto node = std::make_shared<JoyController>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
