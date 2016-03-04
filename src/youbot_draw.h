#ifndef DRAWING_HPP_
#define DRAWING_HPP_

#include <iostream>
#include <ros/ros.h>
#include <brics_actuator/JointPositions.h>
#include <boost/units/systems/si.hpp>

namespace youbot_arm_drawing {

class DrawingController
{
public:
	DrawingController(ros::NodeHandle& nh, std::string name);
	void stepInput();
	ros::Publisher arm_pos_cmd;
private:
	double lr;

};


} // namespace youbot_arm_drawing
#endif // DRAWING_HPP_
