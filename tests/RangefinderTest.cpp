#include <Rangefinder/Rangefinder.h>
#include <iostream>

int main()
{
	std::string port;
	Rangefinder rf;

	std::cout << "Port: "; std::cin >> port;
	rf.open(port);
	std::cout << rf.getRange();

	return 0;
}