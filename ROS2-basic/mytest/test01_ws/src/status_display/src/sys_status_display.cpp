#include "rclcpp/rclcpp.hpp"
#include "status_interfaces/msg/system_status.hpp"
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/subscription.hpp>
#include <QLabel>
#include <QString>
#include <QApplication>

#define LAMBDA 0

class SysStatusDisplay : public rclcpp::Node
{
	public:
		SysStatusDisplay() : Node("sys_status_display")
		{
			// 订阅SystemStatus消息
#if LAMBDA
			subscription_ = this->create_subscription<status_interfaces::msg::SystemStatus>(
				"sys_status", 10,
				[&](const status_interfaces::msg::SystemStatus::SharedPtr msg) -> void {
					label_->setText(get_qstr_from_msg(msg));
				}
			);
#else
			subscription_ = this->create_subscription<status_interfaces::msg::SystemStatus>(
				"sys_status", 10,
				std::bind(&SysStatusDisplay::handle_sys_status, this, std::placeholders::_1)
			);
#endif
			label_ = new QLabel(get_qstr_from_msg(std::make_shared<status_interfaces::msg::SystemStatus>()));
			label_->show();
		}

		void handle_sys_status(const status_interfaces::msg::SystemStatus::SharedPtr msg) {
			std::stringstream show_str;
			show_str
				<< "========SystemStatus==========\n"
				<< "stamp time:\t" << msg->stamp.sec << "\ts\n"
				<< "host name:\t" << msg->host_name << "\t\n"
				<< "cpu percent:\t" << msg->cpu_percent << "\t\n"
				<< "memory percent:\t" << msg->memory_percent << "\t\n"
				<< "memory total:\t" << msg->memory_total << "\t\n"
				<< "memory avaiable:\t" << msg->memory_available << "\t\n"
				<< "net sent:\t" << msg->net_sent << "\t\n"
				<< "net recv:\t" << msg->net_recv << "\t\n"
				<< "==============================\n";
			label_->setText(QString::fromStdString(show_str.str()));
		}

		QString get_qstr_from_msg(const status_interfaces::msg::SystemStatus::SharedPtr msg)
		{
			std::stringstream show_str;
			show_str
				<< "========SystemStatus==========\n"
				<< "stamp time:\t" << msg->stamp.sec << "\ts\n"
				<< "host name:\t" << msg->host_name << "\t\n"
				<< "cpu percent:\t" << msg->cpu_percent << "\t\n"
				<< "memory percent:\t" << msg->memory_percent << "\t\n"
				<< "memory total:\t" << msg->memory_total << "\t\n"
				<< "memory avaiable:\t" << msg->memory_available << "\t\n"
				<< "net sent:\t" << msg->net_sent << "\t\n"
				<< "net recv:\t" << msg->net_recv << "\t\n"
				<< "==============================\n";
			return QString::fromStdString(show_str.str());
		}

	private:
		rclcpp::Subscription<status_interfaces::msg::SystemStatus>::SharedPtr subscription_;
		QLabel *label_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	QApplication app(argc, argv);
	auto node = std::make_shared<SysStatusDisplay>();
	std::thread spin_thread([&]()->void{rclcpp::spin(node);});
	spin_thread.detach();
	app.exec();
	rclcpp::shutdown();
	return 0;
}
