/*
 * node.cpp
 *
 *  Created on: 06.05.2016
 *      Author: je
 */

#include "ros/ros.h"
#include "controller_manager/controller_manager.h"
#include "hexapod_hardware/hexapod_hardware.h"

boost::shared_ptr<Hexapod> hexapod;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hexapod_hardware");

	ros::NodeHandle n;

	Hexapod hexapod(n);
	controller_manager::ControllerManager cm(&hexapod);

	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::Time prev_time = ros::Time::now();
	ros::Rate rate(10);

	while(ros::ok()) {
		const ros::Time time = ros::Time::now();
		const ros::Duration period = time - prev_time;

		hexapod.read(time, period);
		cm.update(time, period);
		hexapod.write(time, period);

		rate.sleep();
	}

	return 0;
}
