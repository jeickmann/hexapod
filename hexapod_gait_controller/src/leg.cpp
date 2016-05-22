/*
 * leg.cpp
 *
 *  Created on: 06.05.2016
 *      Author: je
 */

#include <math.h>
#include <string>
#include "hexapod_gait_controller/leg.h"
#include "std_msgs/Float64.h"

Leg::Leg(const std::string name, ros::NodeHandle & n, bool reflected) : name(name), isReflected(false), stepHeight(0.03) {

	coxa_pub = n.advertise<std_msgs::Float64>(name + "_coxa_controller/command", 10);
	femur_pub = n.advertise<std_msgs::Float64>(name + "_femur_controller/command", 10);
	tibia_pub = n.advertise<std_msgs::Float64>(name + "_tibia_controller/command", 10);

	if(!n.getParam("femur_length", femurLength)) {
		ROS_ERROR("Unable to determine femur_length");
	}


	if(!n.getParam("tibia_length", tibiaLength)) {
		ROS_ERROR("Unable to determine tibia_length");
	}

	if(!n.getParam("coxa_height", coxaHeight)) {
		ROS_ERROR("Unable to determine coxa_height");
	}
}

void Leg::setAnglesRadians(double coxa, double femur, double tibia) {
	std_msgs::Float64 coxa_msg;
	coxa_msg.data = coxa;
	coxa_pub.publish(coxa_msg);

	std_msgs::Float64 femur_msg;
	femur_msg.data = femur;
	femur_pub.publish(femur_msg);

	std_msgs::Float64 tibia_msg;
	tibia_msg.data = tibia;
	tibia_pub.publish(tibia_msg);
}

void Leg::setPosition(const KDL::Vector & p) {
	double z = -p.z() + coxaHeight;

	double coxaAngle = -atan2(p.x(), p.y());

	double G = sqrt(p.x()*p.x() + p.y()*p.y());
	double L = sqrt(z*z + G*G);

	double a1 = acos(z/L);
	double a2 = acos((tibiaLength*tibiaLength-femurLength*femurLength-L*L)/(-2*femurLength*L));
	double femurAngle = a1+a2-M_PI/2;

	double tibiaAngle = -M_PI+acos((L*L-tibiaLength*tibiaLength-femurLength*femurLength)/(-2*tibiaLength*femurLength));

	//ROS_ERROR("a1=%f  %f,%f,%f", a1*180/M_PI, coxaAngle*180/M_PI, femurAngle*180/M_PI, tibiaAngle*180/M_PI);
	setAnglesRadians(coxaAngle, femurAngle, tibiaAngle);
}


void Leg::advancePosition(double t) {
	if(isStance) {
		setPosition(
				interpolateLinearly(aep, pep, t)
			);
	} else {
		setPosition(
				interpolateQuadraticBezier(pep, aep,
							swingControlPoint, t));
	}
}

void Leg::setAEP(const KDL::Vector& p) {
	aep = p;
	if(isReflected) {
		aep.y(-aep.y());
	}
	updateControlPoint();
}

void Leg::setPEP(const KDL::Vector& p) {
	pep = p;
	if(isReflected) {
		pep.y(-pep.y());
	}
	updateControlPoint();
}

void Leg::setStepHeight(double stepHeight) {
	this->stepHeight = stepHeight;
	updateControlPoint();
}

void Leg::updateControlPoint() {
	swingControlPoint = pep + (aep - pep)/2;
	swingControlPoint.z(swingControlPoint.z() + stepHeight);
}

KDL::Vector Leg::interpolateLinearly(const KDL::Vector& startPos, const KDL::Vector& endPos, double t) {
	KDL::Vector delta = endPos - startPos;
	return startPos + delta * t;
}

KDL::Vector Leg::interpolateQuadraticBezier(const KDL::Vector& startPos, const KDL::Vector& endPos,
		const KDL::Vector& controlPoint, double t) {
	KDL::Vector position = (1 - t) * ((1 - t) * startPos + t * controlPoint)
			+ t * ((1 - t) * controlPoint + t * endPos);

	return position;
}
