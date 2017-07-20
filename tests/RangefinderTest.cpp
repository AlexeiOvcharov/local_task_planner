#include <Rangefinder/Rangefinder.h>
#include <iostream>

int main()
{
	Rangefinder rf;
	rf.open("/dev/ttyACM1");
	std::cout << rf.getRange();

	return 0;
}