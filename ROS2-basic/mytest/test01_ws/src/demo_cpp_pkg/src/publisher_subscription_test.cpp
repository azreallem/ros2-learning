#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/publisher_options.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sstream>
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim_msgs/msg/pose.hpp"
#include <chrono>

using namespace std::chrono_literals;

/*
话题通信：即发布者和订阅者通信
发布者将消息发布到某个话题上，订阅者订阅话题即可获得数据，数据是单向传输的。
启动publisher_subscription_test之后，/turtle1/cmd_vel Publisher count + 1, /turtle1/pose Subscription count + 1
发布者发布message消息之后，订阅者会收到该message消息自动进行操作。


$ ros2 topic list -t         
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim_msgs/msg/Color]
/turtle1/pose [turtlesim_msgs/msg/Pose]

$ ros2 topic info /turtle1/cmd_vel
Type: geometry_msgs/msg/Twist
Publisher count: 0 (+1)等待发布者发布操作
Subscription count: 1

$ ros2 topic info /turtle1/pose
Type: turtlesim_msgs/msg/Pose
Publisher count: 1
Subscription count: 0 (+1)等待订阅者订阅其位置
*/

#define PUB 1
#define SUB 0
#define TIMER 1

class PersonNode : public rclcpp::Node
{
	public:
		PersonNode(const std::string node_name) : Node(node_name)
		{
			this->name = node_name;
#if PUB
			// 发布Twist消息
			publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
#endif
#if SUB
			// 订阅Pose消息
			subscription_ = this->create_subscription<turtlesim_msgs::msg::Pose>("/turtle1/pose", 10,
				std::bind(&PersonNode::subscription_callback, this, std::placeholders::_1)
			);
#endif
#if TIMER
			timer_ = this->create_wall_timer(1000ms, std::bind(&PersonNode::timer_callback, this));
#endif
		}

		void test_print(const std::string str)
		{
			RCLCPP_INFO(this->get_logger(), "\n%s", str.c_str());
		}

	private:
		//get pose and then send to cmd_vel
		void subscription_callback(const turtlesim_msgs::msg::Pose::SharedPtr pose) //while
		{
			auto message = geometry_msgs::msg::Twist();
			double current_x = pose->x;
			double current_y = pose->y;
			double distance = std::sqrt(
				(target_x_ - current_x) * (target_x_ - current_x)
				+ (target_y_ - current_y) * (target_y_ - current_y)
			);
			double angle = std::atan2(target_y_ - current_y, target_x_ - current_x) - pose->theta;

			std::stringstream print_str;
			print_str << "current_x: " << current_x  << "\n"
				  << "current_y: " << current_y  << "\n"
				  << "target_x: " << target_x_  << "\n"
				  << "target_y: " << target_y_  << "\n"
				  << "distance: " << distance  << "\n"
				  << "angle: " << angle  << "\n";
			this->test_print(print_str.str());

			// setting angular.z and linear.x ...
			if (distance > 0.1) {
				if (fabs(angle) > 0.2)
					message.angular.z = fabs(angle);
				else
					message.linear.x = k_ * distance;
			}
			if (message.linear.x > max_speed_) {
				message.linear.x = max_speed_;
			}

			publisher_->publish(message); //send to cmd_vel
		}

		void timer_callback()
		{
			auto msg = geometry_msgs::msg::Twist();
			msg.linear.x = 1.0;
			msg.angular.z = 0.5;
			publisher_->publish(msg);
		}

	private:
		std::string name;
#if PUB
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
#endif
#if SUB
		rclcpp::Subscription<turtlesim_msgs::msg::Pose>::SharedPtr subscription_;
#endif
#if TIMER
		rclcpp::TimerBase::SharedPtr timer_;
#endif
		double target_x_{10.0};
		double target_y_{10.0};
		double k_{1.0};
		double max_speed_{3.0};
};

int main(int args, char **argv)
{
	rclcpp::init(args, argv);
	auto node = std::make_shared<PersonNode>("cpp_node_test");
	node->test_print("hello cpp_node_test");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
