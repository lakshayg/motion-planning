#include <algorithm>
#include <iostream>
#include <cstdlib>
#include <cassert>
#include <string>
#include "vrep.hpp"

extern "C" {
	#include "extApi.c"
	#include "extApiPlatform.c"
}

#define CONSOLE_AUTO_CLOSE 1
#define CONSOLE_LINE_WRAP  2
#define CONSOLE_USER_CLOSE 4
#define CONSOLE_HIDE_PAUSE 8
#define CONSOLE_STAY      16

#define MSEC 1
#define SEC 1000

VrepArm::VrepArm(int argc, const char **argv) {
	// connect to V-REP
	ip_addr = "127.0.0.1";
	port = std::stoi(argv[1]);
	timeout = 5*SEC;
	comm_time = 1*MSEC;
	client_id = simxStart(ip_addr.c_str(), port, true, true, timeout, comm_time);

	// start the console
	name = "Console";
	max_lines = 100;
	mode = CONSOLE_AUTO_CLOSE | CONSOLE_LINE_WRAP;
	position = nullptr;
	size = nullptr;
	text_color[0] = 0; text_color[1] = 0; text_color[2] = 0;
	bg_color[0] = 1; bg_color[1] = 1; bg_color[2] = 1;
	simxAuxiliaryConsoleOpen(client_id, name.c_str(), max_lines, mode,
	                        position, size, text_color, bg_color, &ch,
													simx_opmode_blocking);

	// initialize arm joint handles
	dof = argc - 2; // argv = {exe-name, port-num, jh1, jh2, ..., jhn}
	assert(dof > 1);
	jh.resize(dof+1); // jh = {X, jh1, jh2, ..., jhn}
	for(size_t i = 0; i < dof; ++i) {
		jh[i+1] = std::stoi(argv[i+2]);
	}
}

VrepArm::~VrepArm() {
	simxAuxiliaryConsoleClose(client_id, ch, simx_opmode_oneshot);
	simxFinish(client_id);
}

void VrepArm::set_joint_velocities(double *vel) {
	for(size_t i = 1; i <= dof; ++i) {
		simxSetJointTargetVelocity(client_id, jh[i], vel[i-1], simx_opmode_oneshot);
	}
}
void VrepArm::set_joint_velocity(int joint_num, double vel) {
	simxSetJointTargetVelocity(client_id, jh[joint_num], vel, simx_opmode_oneshot);
}

void VrepArm::console_log(std::string str) {
	simxAuxiliaryConsolePrint(client_id, ch, str.c_str(), simx_opmode_blocking);
}
