#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <memory>
#include <chrono>
#include <rclcpp/duration.hpp>
#include <tf2/utils.hpp>


class TFListener : public rclcpp::Node
{
	public:
		TFListener() : Node("tf_listener")
		{
			buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
			listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
			timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&TFListener::getTransform, this));
		}
		void getTransform()
		{
			try {
				const auto transform = buffer_->lookupTransform("base_link", "target_point",
									        this->get_clock()->now(),
										rclcpp::Duration::from_seconds(1.0f));
				const auto &translation = transform.transform.translation;
				const auto &rotation = transform.transform.rotation;
				double roll, pitch, yaw;
				tf2::getEulerYPR(rotation, yaw, pitch, roll);
				RCLCPP_INFO(this->get_logger(), "Translation: [%f, %f, %f]", translation.x, translation.y, translation.z);
				RCLCPP_INFO(this->get_logger(), "Rotation: [%f, %f, %f, %f]", rotation.x, rotation.y, rotation.z, rotation.w);
				RCLCPP_INFO(this->get_logger(), "Euler angles: [%f, %f, %f]", roll, pitch, yaw);
			}
			catch (tf2::TransformException &ex) {
				RCLCPP_WARN(this->get_logger(), "exception: %s", ex.what());
			}
		}

	private:
		rclcpp::TimerBase::SharedPtr timer_;
		std::shared_ptr<tf2_ros::TransformListener> listener_;
		std::shared_ptr<tf2_ros::Buffer> buffer_;

};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TFListener>());
	rclcpp::shutdown();
	return 0;
}

