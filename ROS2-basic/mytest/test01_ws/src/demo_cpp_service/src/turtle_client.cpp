#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/publisher_options.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sstream>
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim_msgs/msg/pose.hpp"
#include <chrono>
#include "srv_interfaces/srv/patrol.hpp"

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

#define TIMER 1
#define SERVICE 0
#define CLIENT 1

class PersonNode : public rclcpp::Node
{
	public:
		PersonNode(const std::string node_name) : Node(node_name)
		{
			this->name = node_name;
#if CLIENT
			// 服务端service
			client_ = this->create_client<srv_interfaces::srv::Patrol>(
				"patrol"
			);
#endif
#if TIMER
			// 定时器timer
			timer_ = this->create_wall_timer(1000ms, std::bind(&PersonNode::timer_callback, this));
#endif
		}

		void test_print(const std::string str)
		{
			RCLCPP_INFO(this->get_logger(), "\n%s", str.c_str());
		}

	private:
		void timer_callback()
		{
			while (!client_->wait_for_service(1s)) {
				if (!rclcpp::ok()) {
					this->test_print("ERROR!");
					return;
				}
				this->test_print("waitting service online ...");
			}
			auto request = std::make_shared<srv_interfaces::srv::Patrol::Request>();
			request->target_x = rand() % 15;
			request->target_y = rand() % 15;
			std::stringstream ss;
			ss << "target(" << request->target_x << "," << request->target_y << ")";
			this->test_print(ss.str());
			client_->async_send_request(
				request,
				[&](rclcpp::Client<srv_interfaces::srv::Patrol>::SharedFuture result_future) -> void {
					auto response = result_future.get();
					if (response->result == srv_interfaces::srv::Patrol::Response::SUCCESS)
						this->test_print("target ok.");
					else if (response->result == srv_interfaces::srv::Patrol::Response::FAIL)
						this->test_print("target failed.");
				}
			);
		}

	private:
		std::string name;
#if TIMER
		rclcpp::TimerBase::SharedPtr timer_;
#endif
#if CLIENT
		rclcpp::Client<srv_interfaces::srv::Patrol>::SharedPtr client_;
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

