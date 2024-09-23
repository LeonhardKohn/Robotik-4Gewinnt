#include <ros/ros.h>
#include <franka_ros_example/frankX_move.h>
#include <franka_ros_example/frankX_gripper.h>
#include <franka_ros_example/frankX_joint.h>

namespace FrankaRobot
{
	bool service_move_call(franka_ros_example::frankX_move::Request& req, franka_ros_example::frankX_move::Response& res)
	{
        ROS_INFO("Move");
		return true;
	}

	bool service_joint_call(franka_ros_example::frankX_joint::Request&req, franka_ros_example::frankX_joint::Response& res)
	{
        ROS_INFO("Joint");
		return true;
	}

	bool service_gripper_call(franka_ros_example::frankX_gripper::Request& req, franka_ros_example::frankX_gripper::Response& res)
	{
        ROS_INFO("Gripper");
		return true;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "frankX_service_node");
	ros::NodeHandle nh;

	ros::ServiceServer service1 = nh.advertiseService("frankX_service_node/frankX_move", &FrankaRobot::service_move_call);
	ros::ServiceServer service2 = nh.advertiseService("frankX_service_node/frankX_gripper", &FrankaRobot::service_gripper_call);
	ros::ServiceServer service3 = nh.advertiseService("frankX_service_node/frankX_joint", &FrankaRobot::service_joint_call);

	ros::spin();

	return 0;
}
