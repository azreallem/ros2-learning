#include  "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class TurtleCircle: public rclcpp::Node
{
	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; //发布者的智能指针
	public:
		explicit TurtleCircle(const std::string &node_name) : Node(node_name) //防止编译器在某些情况下进行隐式类型转换，避免由于构造函数的隐式调用导致的错误或不期望的行为。
		{
			publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10); //10: 队列大小（Depth）：指定队列中可以存储的最大消息数量。
			timer_ = this->create_wall_timer(1000ms, std::bind(&TurtleCircle::timer_callback, this));
		}
	private:
		void timer_callback()
		{
			auto msg = geometry_msgs::msg::Twist();
			msg.linear.x = 1.0;
			msg.angular.z = 0.5;
			publisher_->publish(msg);
		}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TurtleCircle>("turtle_circle");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
