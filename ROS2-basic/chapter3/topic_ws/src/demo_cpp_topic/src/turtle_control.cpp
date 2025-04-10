#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/subscription_base.hpp>
#include "turtlesim_msgs/msg/pose.hpp"

/* 发布者发布位置信息，
** 订阅者更新移动位置。 
*/
class TurtleController : public rclcpp::Node
{
	private:
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;    //发布者的智能指针
		rclcpp::Subscription<turtlesim_msgs::msg::Pose>::SharedPtr pose_subscription_;  //订阅者的智能指针
		double target_x_{10.0};
		double target_y_{10.0};
		double k_{1.0};
		double max_speed_{3.0};


	public:
		TurtleController() : Node("turtle_controller")
		{
			velocity_publisher_  = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
			pose_subscription_ = this->create_subscription<turtlesim_msgs::msg::Pose>("/turtle1/pose", 10,
				std::bind(&TurtleController::on_pose_receviced_, this, std::placeholders::_1)
				);
		}
	private:
		void on_pose_receviced_(const turtlesim_msgs::msg::Pose::SharedPtr pose) //参数：收到数据的共享指针
		{
			auto message = geometry_msgs::msg::Twist();
			
			// 1. 获取当前的位置
			double current_x = pose->x;
			double current_y = pose->y;
			RCLCPP_INFO(this->get_logger(), "Current local: (x=%f, y=%f)", current_x, current_y);

			// 2. 计算当前海龟的位置和目标位置之间的距离和角度差
			double distance =  std::sqrt(std::pow(target_x_ - current_x, 2) + std::pow(target_y_ - current_y, 2));
			double angle = std::atan2(target_y_ - current_y, target_x_ - current_x) - pose->theta;

			// 3. 控制策略
			if (distance > 0.1) {
				if (fabs(angle) > 0.2)
					message.angular.z = fabs(angle);
				else
					message.linear.x = k_ * distance;
			}

			// 4. 限制线速度最大值
			if (message.linear.x > max_speed_) {
				message.linear.x = max_speed_;
			}

			// 5. 发布位置信息
			velocity_publisher_->publish(message);
		}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TurtleController>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
