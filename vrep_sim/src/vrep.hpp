#ifndef VREP_HPP
#define VREP_HPP

#include <algorithm>
#include <iostream>
#include <cstdlib>
#include <cassert>
#include <vector>
#include <string>

#define NON_MATLAB_PARSING
#define MAX_EXT_API_CONNECTIONS 255
extern "C" {
	#include "extApi.h"
	#include "extApiPlatform.h"
}

/**
 * A class that provides a thin, convenience wrapper over the
 * V-REP C/C++ client API for controlling a robot arm in simulator
*/
class VrepArm {
public:
  VrepArm(int argc, const char **argv);
  ~VrepArm();
	void set_joint_velocities(double *vel);
	void set_joint_velocity(int joint_num, double vel);
	void console_log(std::string str);
private:
  // client config
  std::string ip_addr;
  int port, timeout, comm_time;
  int client_id;

  // console config
  std::string name;
  int max_lines;
  int mode;
  simxInt *position;
  simxInt *size;
  simxFloat text_color[3];
  simxFloat bg_color[3];
  simxInt ch; // console handle

  // arm config
	int dof;
	std::vector<int> jh; // object handles for arm joints
};

#endif
