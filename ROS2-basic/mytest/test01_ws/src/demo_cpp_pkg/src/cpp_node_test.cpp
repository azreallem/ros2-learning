#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <rclcpp/node.hpp>
#include <string>

class PersonNode : public rclcpp::Node
{
	public:
		PersonNode(const std::string node_name) : Node(node_name)
		{
			this->name = node_name;
		}
		void test_print(const std::string str)
		{
			RCLCPP_INFO(this->get_logger(), "%s", str.c_str());
		}
	private:
		std::string name;
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
