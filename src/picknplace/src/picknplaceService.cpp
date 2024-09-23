#include "ros/ros.h"
#include "franka_ros_example/frankX_move.h"
#include "franka_ros_example/frankX_gripper.h"
#include "franka_ros_example/frankX_joint.h"
#include <cstdlib>
#include <picknplace/PickNPlace.h>
#include <picknplace/Empty.h>

namespace service_caller
{
  ros::ServiceClient client_move;
  ros::ServiceClient client_gripper;
  ros::ServiceClient client_joint;

  float speed = 0.1;

  bool call_move_srv(float x, float y, float z, float force_lim, float speed, float a, float b, float c, float elbow)
  {
    franka_ros_example::frankX_move srv;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z;
    srv.request.force_limit = force_lim;
    if (speed > 0.2)
      speed = service_caller::speed;
    srv.request.speed = speed;
    srv.request.a = a;
    srv.request.b = b;
    srv.request.c = c;
    srv.request.elbow = elbow;

    return client_move.call(srv);
  }

  bool call_gripper_srv(float clamp_width, float release_width, float move_width, std::string option)
  {
    franka_ros_example::frankX_gripper srv;
    srv.request.clamp_width = clamp_width;
    srv.request.release_width = release_width;
    srv.request.move_width = move_width;
    if (option != "move" && option != "release" && option != "clamp") // Check valid options
      return false;
    srv.request.option = option;

    return client_gripper.call(srv);
  }

  bool call_joint_srv(float j0, float j1, float j2, float j3, float j4, float j5, float j6, float speed)
  {
    franka_ros_example::frankX_joint srv;
    srv.request.q0 = j0;
    srv.request.q1 = j1;
    srv.request.q2 = j2;
    srv.request.q3 = j3;
    srv.request.q4 = j4;
    srv.request.q5 = j5;
    srv.request.q6 = j6;
    if (speed > 0.2)
      speed = service_caller::speed;
    srv.request.speed = speed;

    return client_joint.call(srv);
  }

  bool call_move_srv(float x, float y, float z, float a, float b, float c)
  {
    return call_move_srv(x, y, z, 5.0f, speed, a, b, c, 0.0f);
  }

  bool moveHome(float speed)
  {
    return service_caller::call_joint_srv(0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4, speed);
  }

  bool picknplaceBlock(float blockX, float blockY, float blockZ,
                       float blockA, float blockB, float blockC,
                       float dropX, float dropY, float dropZ,
                       float dropA, float dropB, float dropC)
  {
    float extraHeight = 0.1;

    ROS_INFO("Moving to default position");
    if (!moveHome(speed))
    {
      return false;
    }
    ROS_INFO("Opening Gripper");
    if (!service_caller::call_gripper_srv(0, 0.08, 0, "release"))
    {
      return false;
    }
    ROS_INFO("Moving above pickup");
    if (!service_caller::call_move_srv(blockX, blockY, blockZ + extraHeight,
                                       blockA, blockB, blockC))
    {
      return false;
    }
    ROS_INFO("Moving to pickup");
    if (!service_caller::call_move_srv(blockX, blockY, blockZ,
                                       blockA, blockB, blockC))
    {
      return false;
    }
    ROS_INFO("Closing Gripper");
    if (!service_caller::call_gripper_srv(0.0, 0, 0, "clamp"))
    {
      return false;
    }
    ROS_INFO("Moving up");
    if (!service_caller::call_move_srv(blockX, blockY, blockZ + extraHeight,
                                       blockA, blockB, blockC))
    {
      return false;
    }
    ROS_INFO("Moving above dropoff");
    if (!service_caller::call_move_srv(dropX, dropY, dropZ + extraHeight,
                                       dropA, dropB, dropC))
    {
      return false;
    }
    ROS_INFO("Moving to dropoff");
    if (!service_caller::call_move_srv(dropX, dropY, dropZ,
                                       dropA, dropB, dropC))
    {
      return false;
    }
    // ros::Duration(1, 0).sleep();
    ROS_INFO("Opening Gripper");
    if (!service_caller::call_gripper_srv(0, 0.08, 0, "release"))
    {
      return false;
    }
    ROS_INFO("Moving upwards");
    if (!service_caller::call_move_srv(dropX, dropY, dropZ + extraHeight,
                                       dropA, dropB, dropC))
    {
      return false;
    }
    ROS_INFO("Move to default position");
    if (!moveHome(speed))
    {
      return false;
    }

    return true;
  }

  bool picknplaceBlock(picknplace::PickNPlace::Request &req, picknplace::PickNPlace::Response &res)
  {
    ROS_INFO("Start Block Move");
    if (picknplaceBlock(
            req.blockX, req.blockY, req.blockZ,
            req.blockA, req.blockB, req.blockC,
            req.dropX, req.dropY, req.dropZ,
            req.dropA, req.dropB, req.dropC))
    {
      ROS_INFO("Finished Block Move");
      return true;
    }
    else
    {
      ROS_ERROR("Failed moving block!\nABORTED!");
      return false;
    }
  }

  bool moveHome(picknplace::Empty::Request &req, picknplace::Empty::Response &res)
  {
    ROS_INFO("Moving to home position");
    if (moveHome(speed))
    {
      ROS_INFO("Finished moving Home");
    }
    else
    {
      ROS_ERROR("Failed moving Home!\nABORTED!");
    }
    return true;
  }

}

// Adapted from http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
int main(int argc, char **argv)
{
  ros::init(argc, argv, "picknplaceService");

  ros::NodeHandle n;
  service_caller::client_move = n.serviceClient<franka_ros_example::frankX_move>("/frankX_service_node/frankX_move");
  service_caller::client_gripper = n.serviceClient<franka_ros_example::frankX_gripper>("/frankX_service_node/frankX_gripper");
  service_caller::client_joint = n.serviceClient<franka_ros_example::frankX_joint>("/frankX_service_node/frankX_joint");

  ros::ServiceServer picknPlace = n.advertiseService("picknplace", service_caller::picknplaceBlock);
  ros::ServiceServer moveHomeService = n.advertiseService("movehome", service_caller::moveHome);

  ROS_INFO("Ready");
  ros::spin();

  return 0;
}