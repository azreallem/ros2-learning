#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <memory>
#include <chrono>


class DynamicTFBroadcaster : public rclcpp::Node
{
	public:
		DynamicTFBroadcaster() : Node("dynamic_tf_broadcaster")
		{
			broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
			timer_ = this->create_wall_timer(std::chrono::microseconds(10), std::bind(&DynamicTFBroadcaster::publish_tf, this));
		}
		void publish_tf()
		{
			geometry_msgs::msg::TransformStamped transform;
			transform.header.stamp = this->get_clock()->now();
			transform.header.frame_id = "map";
			transform.child_frame_id = "base_link";
			transform.transform.translation.x = 2.0;
			transform.transform.translation.y = 3.0;
			transform.transform.translation.z = 0.0;
			tf2::Quaternion quat;
			quat.setRPY(0, 0, 30 * M_PI / 180);              // 弧度制欧拉角转四元数
			transform.transform.rotation = tf2::toMsg(quat); // 四元数转ROS消息类型
			broadcaster_->sendTransform(transform);
		}

	private:
		std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
		rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DynamicTFBroadcaster>());
	rclcpp::shutdown();
	return 0;
}

