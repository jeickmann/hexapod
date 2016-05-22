/*
 * hexapod_hardware.h
 *
 *  Created on: 17.05.2016
 *      Author: je
 */

#ifndef HEXAPOD_HARDWARE_INCLUDE_HEXAPOD_HARDWARE_HEXAPOD_HARDWARE_H_
#define HEXAPOD_HARDWARE_INCLUDE_HEXAPOD_HARDWARE_HEXAPOD_HARDWARE_H_

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <hardware_interface/robot_hw.h>
#include "serial/serial.h"
#include "ros/ros.h"

class Hexapod: public hardware_interface::RobotHW {
public:
	Hexapod(ros::NodeHandle& nh);
	~Hexapod();
	void read(ros::Time time, ros::Duration period);
	void write(ros::Time time, ros::Duration period);

private:
	hardware_interface::JointStateInterface jnt_state_interface;
	hardware_interface::PositionJointInterface jnt_pos_interface;
	joint_limits_interface::PositionJointSaturationInterface jnt_limits_interface;
	double cmd[32];
	double pos[32];
	double vel[32];
	double eff[32];

	std::string port;
	std::vector<std::string> joints;
	std::vector<double> offsets;
	std::vector<int> dirs;
	boost::shared_ptr<serial::Serial> serialPort;
	unsigned int getTiming(double angle);
};

#endif /* HEXAPOD_HARDWARE_INCLUDE_HEXAPOD_HARDWARE_HEXAPOD_HARDWARE_H_ */
