/*
 * hexapod_gait.cpp
 *
 *  Created on: 10.05.2016
 *      Author: je
 */


#include "hexapod_gait_controller/hexapod_gait.h"


Hexapod_Gait::Hexapod_Gait(ros::NodeHandle & n) :
	fl("fl", n, false),
	ml("ml", n, false),
	rl("rl", n, false),
	fr("fr", n, true),
	mr("mr", n, true),
	rr("rr", n, true),
	stepDuration(0),
	currentT(0),
	stepLength(0.04),
	walkingSpread(0.08),
	walkingHeight(0.03),
	outerLegOffset(0.03)
{
	KDL::Vector pep(-stepLength/2, walkingSpread, -walkingHeight);
	KDL::Vector aep(stepLength/2, walkingSpread, -walkingHeight);
	fl.setAEP(aep - KDL::Vector(outerLegOffset, 0, 0));
	fl.setPEP(pep - KDL::Vector(outerLegOffset, 0, 0));
	fl.isStance = true;

	ml.setAEP(aep);
	ml.setPEP(pep);
	ml.isStance = false;

	rl.setAEP(aep + KDL::Vector(outerLegOffset, 0, 0));
	rl.setPEP(pep + KDL::Vector(outerLegOffset, 0, 0));
	rl.isStance = true;

	fr.setAEP(aep - KDL::Vector(outerLegOffset, 0, 0));
	fr.setPEP(pep - KDL::Vector(outerLegOffset, 0, 0));
	fr.isStance = false;

	mr.setAEP(aep);
	mr.setPEP(pep);
	mr.isStance = true;

	rr.setAEP(aep + KDL::Vector(outerLegOffset, 0, 0));
	rr.setPEP(pep + KDL::Vector(outerLegOffset, 0, 0));
	rr.isStance = false;
}

void Hexapod_Gait::setTwist(const geometry_msgs::Twist::ConstPtr& msg) {
    stepDuration = stepLength / msg->linear.x;
    stepDuration = (stepDuration>=1)?stepDuration:1;
}

void Hexapod_Gait::update(const ros::TimerEvent & e) {
	if(stepDuration > 0) {
		if(!e.last_real.isZero()) {
			currentT += (e.current_real.toSec() - e.last_real.toSec()) / stepDuration;
		}

		//step done? if so reverse mode for all legs
		if(currentT > 1) {
			currentT = 0;
			fl.isStance = rl.isStance = mr.isStance = !fl.isStance;
			fr.isStance = rr.isStance = ml.isStance = !fr.isStance;
		}

		fl.advancePosition(currentT);
		ml.advancePosition(currentT);
		rl.advancePosition(currentT);
		fr.advancePosition(currentT);
		mr.advancePosition(currentT);
		rr.advancePosition(currentT);
	}
}
