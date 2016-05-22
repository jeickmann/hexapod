/*
 * hexapod_gait.h
 *
 *  Created on: 10.05.2016
 *      Author: je
 */

#ifndef HEXAPOD_GAIT_CONTROLLER_INCLUDE_HEXAPOD_GAIT_CONTROLLER_HEXAPOD_GAIT_H_
#define HEXAPOD_GAIT_CONTROLLER_INCLUDE_HEXAPOD_GAIT_CONTROLLER_HEXAPOD_GAIT_H_
#include "leg.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <urdf/model.h>


class Hexapod_Gait {
public:
	Hexapod_Gait(ros::NodeHandle & n);
	void setTwist(const geometry_msgs::Twist::ConstPtr& msg);
	void update(const ros::TimerEvent & e);

private:
	void setStance(unsigned int legNumber, double t);

	Leg fl;
	Leg ml;
	Leg rl;

	Leg fr;
	Leg mr;
	Leg rr;

	double stepDuration;
	double currentT;

	double stepLength;
	double walkingSpread;
	double walkingHeight;
	double outerLegOffset;
};

#endif /* HEXAPOD_GAIT_CONTROLLER_INCLUDE_HEXAPOD_GAIT_CONTROLLER_HEXAPOD_GAIT_H_ */
