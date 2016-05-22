/*
 * leg.h
 *
 *  Created on: 06.05.2016
 *      Author: je
 */

#ifndef HEXAPOD_GAIT_CONTROLLER_INCLUDE_HEXAPOD_GAIT_CONTROLLER_LEG_H_
#define HEXAPOD_GAIT_CONTROLLER_INCLUDE_HEXAPOD_GAIT_CONTROLLER_LEG_H_

#include <string>
#include "ros/ros.h"
#include "kdl/frames.hpp"

#include <urdf/model.h>
class Hexapod_Gait;

class Leg {
	friend class Hexapod_Gait;
public:
	Leg(const std::string name, ros::NodeHandle & n, bool reflected = false);
	void setAnglesRadians(double coxa, double femur, double tibia);
	void setPosition(const KDL::Vector & p);

	void advancePosition(double t);

	void setAEP(const KDL::Vector & p);
	void setPEP(const KDL::Vector & p);

	void setStepHeight(double stepHeight);

	bool isStance;

private:
	void updateControlPoint();
	KDL::Vector interpolateLinearly(const KDL::Vector& startPos, const KDL::Vector& endPos, double t);
	KDL::Vector interpolateQuadraticBezier(const KDL::Vector& startPos, const KDL::Vector& endPos,
			const KDL::Vector& controlPoint, double t);
	const std::string name;
	bool isReflected;
	ros::Publisher coxa_pub;
	ros::Publisher femur_pub;
	ros::Publisher tibia_pub;

	double femurLength;
	double tibiaLength;
	double coxaHeight;
	double stepHeight;


	KDL::Vector aep;
	KDL::Vector pep;
	KDL::Vector swingControlPoint;
};



#endif /* HEXAPOD_GAIT_CONTROLLER_INCLUDE_HEXAPOD_GAIT_CONTROLLER_LEG_H_ */
