#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <brics_actuator/JointPositions.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_generator/CStoCS_Pitch.h>
#include <ik_solver_service/SolvePreferredPitchIK.h>
#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>
#include <Eigen/Dense>

static const uint NUM_ARM_JOINTS = 5;
double HOME_POS[] = {0.05, 0.05, -0.05, 0.05, 0.1107};
ros::Publisher armPositionsPublisher;

brics_actuator::JointPositions generateJointPositionMsg(double* joints)
{
  brics_actuator::JointPositions joint_position_msg;

  std::stringstream jointName;
  joint_position_msg.positions.clear();

  for (int i = 0; i < NUM_ARM_JOINTS; i++)
  {
    brics_actuator::JointValue joint;

    joint.value = joints[i];
    joint.unit = boost::units::to_string(boost::units::si::radians);
    jointName.str("");
    jointName << "arm_joint_" << (i + 1);
    joint.joint_uri = jointName.str();

    joint_position_msg.positions.push_back(joint);
  }

  return joint_position_msg;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "draw_square_test");
	ros::NodeHandle nh;
	ros::Rate rate(50);

	// declare position command publisher
	armPositionsPublisher = nh.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);
	brics_actuator::JointPositions arm_position_cmd;
	// declare trajectory generator service
	ros::ServiceClient traj_client = nh.serviceClient<trajectory_generator::CStoCS_Pitch>("From_CS_to_CS_Pitch");

	double max_vel = 0.05;
	double max_acc = 0.5;
	ROS_INFO("max_vel: %f \t max_acc :%f", max_vel, max_acc);
	double first_pt[NUM_ARM_JOINTS]; // first point of the trajectory
	double cur_joint_val[NUM_ARM_JOINTS];
	bool feasible = true;
	trajectory_msgs::JointTrajectory traj, temp;
	trajectory_msgs::JointTrajectoryPoint point;
	trajectory_generator::CStoCS_Pitch traj_srv;

	// define the square to draw
	geometry_msgs::Point pt1, pt2, pt3, pt4;
	double pitch = 0.0;
	pt1.x = 0.386;
	pt1.y = 0.0;
	pt1.z = 0.3;

	pt2.x = 0.386;
	pt2.y = -0.1;
	pt2.z = 0.3;

	pt3.x = 0.386;
	pt3.y = -0.1;
	pt3.z = 0.25;

	pt4.x = 0.386;
	pt4.y = 0.0;
	pt4.z = 0.25;

	traj_srv.request.start_pos = pt1;
	traj_srv.request.end_pos = pt2;
	traj_srv.request.start_pitch = pitch;
	traj_srv.request.end_pitch = pitch;
	traj_srv.request.start_vel = 0.0;
	traj_srv.request.end_vel = 0.0;
	traj_srv.request.max_vel = max_vel;
	traj_srv.request.max_acc = max_acc;

	if (traj_client.call(traj_srv))
	{
		if (traj_srv.response.feasible)
	    {
	      std::cout << "1st segment feasible" << std::endl;
	      temp = traj_srv.response.trajectory;
	      while (!temp.points.empty())
	      {
	        point = temp.points.back();
	        temp.points.pop_back();
	        traj.points.insert(traj.points.begin(), point);
	      }
	      traj.joint_names = temp.joint_names;
	    }
	    else
	    {
	      std::cout << "1st segment Not Feasible" << std::endl;
	      feasible = false;
	    }
	}
	else
	{
		ROS_ERROR("Could not call service.");
	    return 1;
	}

	traj_srv.request.start_pos = pt2;
	traj_srv.request.end_pos = pt3;
	traj_srv.request.start_pitch = pitch;
	traj_srv.request.end_pitch = pitch;
	traj_srv.request.start_vel = 0.0;
	traj_srv.request.end_vel = 0.0;
	traj_srv.request.max_vel = max_vel;
	traj_srv.request.max_acc = max_acc;

	if (traj_client.call(traj_srv))
	{
		if (traj_srv.response.feasible)
	    {
	      std::cout << "2nd segment feasible" << std::endl;
	      temp = traj_srv.response.trajectory;
	      while (!temp.points.empty())
	      {
	        point = temp.points.back();
	        temp.points.pop_back();
	        traj.points.insert(traj.points.begin(), point);
	      }
	      traj.joint_names = temp.joint_names;
	    }
	    else
	    {
	      std::cout << "2nd segment Not Feasible" << std::endl;
	      feasible = false;
	    }
	}
	else
	{
		ROS_ERROR("Could not call service.");
	    return 1;
	}

	traj_srv.request.start_pos = pt3;
	traj_srv.request.end_pos = pt4;
	traj_srv.request.start_pitch = pitch;
	traj_srv.request.end_pitch = pitch;
	traj_srv.request.start_vel = 0.0;
	traj_srv.request.end_vel = 0.0;
	traj_srv.request.max_vel = max_vel;
	traj_srv.request.max_acc = max_acc;

	if (traj_client.call(traj_srv))
	{
		if (traj_srv.response.feasible)
	    {
	      std::cout << "3rd segment feasible" << std::endl;
	      temp = traj_srv.response.trajectory;
	      while (!temp.points.empty())
	      {
	        point = temp.points.back();
	        temp.points.pop_back();
	        traj.points.insert(traj.points.begin(), point);
	      }
	      traj.joint_names = temp.joint_names;
	    }
	    else
	    {
	      std::cout << "3rd segment Not Feasible" << std::endl;
	      feasible = false;
	    }
	}
	else
	{
		ROS_ERROR("Could not call service.");
	    return 1;
	}

	traj_srv.request.start_pos = pt4;
	traj_srv.request.end_pos = pt1;
	traj_srv.request.start_pitch = pitch;
	traj_srv.request.end_pitch = pitch;
	traj_srv.request.start_vel = 0.0;
	traj_srv.request.end_vel = 0.0;
	traj_srv.request.max_vel = max_vel;
	traj_srv.request.max_acc = max_acc;

	if (traj_client.call(traj_srv))
	{
		if (traj_srv.response.feasible)
	    {
	      std::cout << "4th segment feasible" << std::endl;
	      temp = traj_srv.response.trajectory;
	      while (!temp.points.empty())
	      {
	        point = temp.points.back();
	        temp.points.pop_back();
	        traj.points.insert(traj.points.begin(), point);
	      }
	      traj.joint_names = temp.joint_names;
	    }
	    else
	    {
	      std::cout << "4th segment Not Feasible" << std::endl;
	      feasible = false;
	    }
	}
	else
	{
		ROS_ERROR("Could not call service.");
	    return 1;
	}

	// go to start pose
	if (!feasible)
	{
		ROS_WARN("Trajectory is not feasible! Quit");
		return 1;
	}
	point = traj.points.back();
	int i = 0;
	while (!point.positions.empty())
	{
		first_pt[i] = point.positions.back();
	    i++;
	    point.positions.pop_back();
	}
	ROS_INFO("Move to ready pose.");

	ros::spinOnce();
	i = 0;
	while (nh.ok() && i < 5)
	{
		armPositionsPublisher.publish(generateJointPositionMsg(first_pt));
		i++;
		ros::spinOnce();
		rate.sleep();
	}

    ROS_INFO("READY TO DRAW?");
    int x;
    std::cin >> x;

    while (nh.ok() && (!traj.points.empty()))
    {
    	// retrieve current joint value
    	point = traj.points.back();
    	traj.points.pop_back();
    	for(int k = 0; k < 5; k++)
    	{
    		cur_joint_val[k] = point.positions.back();
    		point.positions.pop_back();
    	}

    	// send command to youbot
    	armPositionsPublisher.publish(generateJointPositionMsg(cur_joint_val));
    	rate.sleep();
    }

	sleep(5.0);
	// go back to home position
	armPositionsPublisher.publish(generateJointPositionMsg(HOME_POS));

	return 0;
}
