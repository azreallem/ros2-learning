#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <string>

class PersonNode : public rclcpp::Node
{
	private:
		std::string name_;
		int age_;
	public:
		PersonNode(const std::string &node_name,
				const std::string &name,
				const int &age) : Node(node_name) {
			this->name_ = name;
			this->age_ = age;
		}
		void eat(const std::string &food_name) {
			RCLCPP_INFO(this->get_logger(),
				"My name is %s. I'm %d years old."
				"I'm eating %s.",
				name_.c_str(), age_, food_name.c_str());
		}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PersonNode>("cpp_node", "Zhang San", 18);
	node->eat("fish");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
