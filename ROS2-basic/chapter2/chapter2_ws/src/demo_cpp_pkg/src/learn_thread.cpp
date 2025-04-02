#include <chrono>
#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <cpp-httplib/httplib.h>

class Download
{
	public:
		void download(const std::string &host,
				const std::string &path,
				const std::function<void(const std::string &, const std::string &)> &callback)
		{
			std::cout << "thread ID: " << std::this_thread::get_id() << std::endl;
			httplib::Client client(host);
			auto response = client.Get(path);
			if (response && response->status == 200) {
				callback(path, response->body);
			} else {
				std::cout << "http response Error." << std::endl;
			}
		}

		void start_download(const std::string &host,
				const std::string &path,
				const std::function<void(const std::string &, const std::string &)> &callback)
		{
			auto download_fun = std::bind(&Download::download, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
			std::thread download_thread(download_fun, host, path, callback);
			download_thread.detach();
		}
};

int main()
{
	Download download;
	auto download_finish_callback = [](const std::string &path, const std::string &result) -> void {
		std::cout << "Download finish: " << path << "Total: "
			<< result.length() << "B, context: "
			<< result.substr(0, 16) << std::endl;

		std::string save_path("/home/gaoliang/tmp/tmp.png");
		std::ofstream file(save_path, std::ios::binary);  // Open file in binary mode
		if (file.is_open()) {
			file.write(result.data(), result.size());  // Write result to the file
			file.close();
			std::cout << "File saved at: " << save_path << std::endl;
		} else {
			std::cerr << "Failed to open file for writing: " << save_path << std::endl;
		}
	};

	std::string host = "http://blog.azreallem.top";
	std::string path[3] = {
		"/usr/uploads/2023/10/2462860770.png",
		"/usr/uploads/2023/02/633820335.png",
		"/usr/uploads/2023/02/3521234345.png",
	};

	download.start_download(host, path[0], download_finish_callback);
	download.start_download(host, path[1], download_finish_callback);
	download.start_download(host, path[2], download_finish_callback);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000 * 10));
	return 0;
}
