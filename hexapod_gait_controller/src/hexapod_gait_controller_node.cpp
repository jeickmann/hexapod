/*
 * node.cpp
 *
 *  Created on: 06.05.2016
 *      Author: je
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "hexapod_gait_controller/hexapod_gait.h"
#include "hexapod_gait_controller/leg.h"

boost::shared_ptr<Hexapod_Gait> gait_controller;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hexapod_gait_controller");

	ros::NodeHandle n;

	gait_controller = boost::shared_ptr<Hexapod_Gait>(new Hexapod_Gait(n));

	ros::Subscriber sub = n.subscribe("cmd_vel", 10, &Hexapod_Gait::setTwist, gait_controller);
	if(!sub) {
		ROS_ERROR("Unable to subscribe");
	} else {
		ROS_ERROR("SUBSCRIBE");
	}

	ros::Timer timer = n.createTimer(ros::Duration(0.02), boost::bind(&Hexapod_Gait::update, gait_controller, _1));

	ros::spin();
}
