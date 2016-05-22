/*
 * hexapod_hardware.cpp
 *
 *  Created on: 17.05.2016
 *      Author: je
 */

#include "hexapod_hardware/hexapod_hardware.h"
#include <joint_limits_interface/joint_limits_urdf.h>
#include <urdf/model.h>
#include <vector>
#include <string>

#define MIN_PULSE_WIDTH       720
#define MAX_PULSE_WIDTH      2300
#define SERVO_RANGE			(155.0/180.0*M_PI)

Hexapod::Hexapod(ros::NodeHandle& nh) {
	nh.getParam("hw_joints", joints);
	nh.getParam("comport", port);
	//if no offsets given, set them all to 0
	if(!nh.getParam("hw_offsets", offsets)) {
		offsets.resize(joints.size(), 0.0);
	}
	if(!nh.getParam("hw_dirs", dirs)) {
		dirs.resize(joints.size(), 1);
	}
	ROS_ERROR("Configuring %i joints", joints.size());
	urdf::Model urdf;
	urdf.initParam("robot_description");

	// Data structures
	joint_limits_interface::JointLimits limits;
	joint_limits_interface::SoftJointLimits soft_limits;

	for(unsigned int i=0;i<joints.size();i++) {

		// connect and register the joint state interface
		hardware_interface::JointStateHandle state_handle(joints[i], &pos[i], &vel[i],
				&eff[i]);
		jnt_state_interface.registerHandle(state_handle);

		// connect and register the joint position interface
		hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joints[i]), &cmd[i]);
		jnt_pos_interface.registerHandle(pos_handle);

		boost::shared_ptr<const urdf::Joint> urdf_joint = urdf.getJoint(joints[i]);
		if(!getJointLimits(urdf_joint, limits)) {
			ROS_ERROR("Could not get joint limits for joint %s", joints[i].c_str());
		}

		joint_limits_interface::PositionJointSaturationHandle limits_handle(jnt_pos_interface.getHandle(joints[i]), limits);
		jnt_limits_interface.registerHandle(limits_handle);
	}

	registerInterface(&jnt_state_interface);
	registerInterface(&jnt_pos_interface);


	serialPort.reset(new serial::Serial(port, 115200, serial::Timeout::simpleTimeout(1000)));
}

Hexapod::~Hexapod() {
}

void Hexapod::read(ros::Time time, ros::Duration period) {
	//so far, do nothing
}

void Hexapod::write(ros::Time time, ros::Duration period) {
	//jnt_limits_interface.enforceLimits(period);
	std::stringstream cmdString;
	for(unsigned int i=0;i<joints.size();i++) {
		double hardware_pos = (cmd[i]+offsets[i])*dirs[i];
		unsigned int power = getTiming(hardware_pos);
		cmdString << "#" << i << "P" << power;
		pos[i] = cmd[i];
		if(i%10 == 0 || i == joints.size()-1) {
			cmdString << "\r\n";
			serialPort->write(cmdString.str());
			cmdString.str("");
		}
	}
}

unsigned int Hexapod::getTiming(double angle) {
	//change M_PI to whatever the range of the servos really is with those Pulse-Widths
	return static_cast<unsigned int>(((angle+SERVO_RANGE/2) / SERVO_RANGE) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) + MIN_PULSE_WIDTH);
}
