#include <iostream>
#include <memory>
#include <ostream>
int main()
{
	auto p1 = std::make_shared<std::string>("This is a str.");
	std::cout << "p1's count: " << p1.use_count() << ", addr: " <<
		p1.get() << std::endl;

	auto p2 = p1;
	std::cout << "p1's count: " << p1.use_count() << ", addr: " <<
		p1.get() << std::endl;
	std::cout << "p2's count: " << p2.use_count() << ", addr: " <<
		p2.get() << std::endl;


	p1.reset();
	std::cout << "p1's count: " << p1.use_count() << ", addr: " <<
		p1.get() << std::endl;
	std::cout << "p2's count: " << p2.use_count() << ", addr: " <<
		p2.get() << std::endl;
	std::cout << "p2's sources: " << p2->c_str() << std::endl;

	return 0;
}
