#include "vrep.hpp"

using namespace std;

int main(int argc, char const *argv[]) {
	VrepArm arm(argc, argv);
	arm.console_log("Successfully connected to V-REP arm\n");
	return 0;
}
