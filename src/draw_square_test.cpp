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

std::vector<geometry_msgs::Point> getOctagon(geometry_msgs::Point center, double radius)
{
	std::vector<geometry_msgs::Point> out;
	out.resize(8);

	double half_length = radius * sin(M_PI_4*0.5);
	out[0].x = center.x;
	out[0].y = center.y + half_length;
	out[0].z = center.z + half_length*(1 + sqrt(2.0));

	out[1].x = center.x;
	out[1].y = center.y - half_length;
	out[1].z = out[0].z;

	out[2].x = center.x;
	out[2].y = center.y - half_length*(1 + sqrt(2.0));
	out[2].z = center.z + half_length;

	out[3].x = center.x;
	out[3].y = out[2].y;
	out[3].z = center.z - half_length;

	out[4].x = center.x;
	out[4].y = out[1].y;
	out[4].z = center.z - half_length*(1 + sqrt(2.0));

	out[5].x = center.x;
	out[5].y = out[0].y;
	out[5].z = out[4].z;

	out[6].x = center.x;
	out[6].y = center.y + half_length*(1 + sqrt(2.0));
	out[6].z = out[3].z;

	out[7].x = center.x;
	out[7].y = out[6].y;
	out[7].z = out[2].z;

	return out;
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


	geometry_msgs::Point center,pt_tmp;
	std::vector<geometry_msgs::Point> pts;
	double pitch = 0.0;
	double radius = 0.08;
	center.x = 0.4;
	center.y = 0.0;
	center.z = 0.33;
	// define the outer octagon
	pts = getOctagon(center, radius);
	pts.push_back(pts[0]);
	//pull out from drawing surface
	pt_tmp = pts[0];
	pt_tmp.x -= 0.01;
	pts.push_back(pt_tmp);

	// define the inner octagon
	std::vector<geometry_msgs::Point> pts_tmp;
	pts_tmp = getOctagon(center, radius*0.85);
	// distance between outer and inner octagon
	double diff = radius*0.15*cos(M_PI_4*0.5);
	double half_length_inner = 0.75*radius * sin(M_PI_4*0.5);
	pt_tmp = pts_tmp[0];
	pt_tmp.x -= 0.01;
	pts.push_back(pt_tmp);

	pts.insert(pts.end(), pts_tmp.begin(), pts_tmp.begin()+3);

	pt_tmp = pts_tmp[2];
	pt_tmp.y = pts_tmp[1].y + diff;
	pts.push_back(pt_tmp);

	pt_tmp.z = pts_tmp[3].z;
	pts.push_back(pt_tmp);

	pt_tmp.y = pts_tmp[5].y - diff;
	pts.push_back(pt_tmp);

	pt_tmp.z = pts_tmp[7].z;
	pts.push_back(pt_tmp);

	pts.push_back(pts_tmp[7]);
	pts.push_back(pts_tmp[0]);

	pt_tmp = pts_tmp[0];
	pt_tmp.x -= 0.01;
	pts.push_back(pt_tmp);

	pt_tmp = pts_tmp[7];
	pt_tmp.x -= 0.01;
	pt_tmp.z -= diff;
	pts.push_back(pt_tmp);

	pt_tmp.x = pts_tmp[7].x;
	pts.push_back(pt_tmp);

	pt_tmp.y = pts_tmp[0].y;
	pts.push_back(pt_tmp);

	pt_tmp.z = pts_tmp[3].z - diff;
	pts.push_back(pt_tmp);

	pt_tmp.y = pts_tmp[1].y;
	pts.push_back(pt_tmp);

	pt_tmp.z = pts_tmp[2].z - diff;
	pts.push_back(pt_tmp);

	pt_tmp.y = pts_tmp[2].y;
	pts.push_back(pt_tmp);

	for(int i = 0; i < 4; i++)
	{
		pts.push_back(pts_tmp[i+3]);
	}

	pt_tmp.y = pts_tmp[7].y;
	pts.push_back(pt_tmp);

	pt_tmp.x -= 0.01;
	pts.push_back(pt_tmp);


	double max_vel = 0.05;
	double max_acc = 0.5;
	ROS_INFO("max_vel: %f \t max_acc :%f", max_vel, max_acc);
	double first_pt[NUM_ARM_JOINTS]; // first point of the trajectory
	double cur_joint_val[NUM_ARM_JOINTS];
	bool feasible = true;
	trajectory_msgs::JointTrajectory traj, temp;
	trajectory_msgs::JointTrajectoryPoint point;
	trajectory_generator::CStoCS_Pitch traj_srv;

	ROS_INFO("Generating trajectories, please wait...");
	for (int i = 0; i < pts.size()-1; i++)
	{
		traj_srv.request.start_pos = pts[i];
		if (i < (pts.size()-1))
		{
			traj_srv.request.end_pos = pts[i+1];
		}
		else
		{
			traj_srv.request.end_pos = pts[0];
		}
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
//		      std::cout << "segment feasible" << std::endl;
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
		      ROS_WARN("Segment %d Not Feasible", i+1);
		      feasible = false;
		    }
		}
		else
		{
			ROS_ERROR("Could not call service.");
		    return 1;
		}
	}

	// go to start pose
	if (!feasible)
	{
		ROS_WARN("Trajectory is not feasible! Quit");
		return 1;
	}
	ROS_INFO("Done");

	point = traj.points.back();
	int i = 0;
	while (!point.positions.empty())
	{
		first_pt[i] = point.positions.back();
	    i++;
	    point.positions.pop_back();
	}
	ROS_INFO("Move to ready pose");

	ros::spinOnce();
	i = 0;
	while (nh.ok() && i < 5)
	{
		armPositionsPublisher.publish(generateJointPositionMsg(first_pt));
		i++;
		ros::spinOnce();
		rate.sleep();
	}

    ROS_INFO("Ready to draw?");
    int x;
    std::cin >> x;

    ROS_INFO("Start drawing...");
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
    ROS_INFO("Done");

	sleep(5.0);
	// go back to home position
	ROS_INFO("Back to home position");
	armPositionsPublisher.publish(generateJointPositionMsg(HOME_POS));

	return 0;
}
