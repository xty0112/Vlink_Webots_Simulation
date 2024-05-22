#include <webots/Emitter.hpp>
#include <webots/Field.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

#include <stdlib.h>
#include <cstring>
#include <iostream>
#include <string>

using namespace std;
using namespace webots;

class Driver : public Supervisor {
public:
	Driver();
	void run();

private:
	int timeStep;
};
Driver::Driver() {
	timeStep = 128;
}
void Driver::run()
{

}
int main() {
	Driver* controller = new Driver();
	controller->run();
	delete controller;
	return 0;
}