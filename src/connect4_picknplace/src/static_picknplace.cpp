#include "ros/ros.h"
#include <picknplace/PickNPlace.h>
#include <connect4_picknplace/connect4_picknplace.h>
#include <picknplace/Empty.h>
#include <connect4_picknplace/singleCoords.h>
#include <franka_ros_example/frankX_move.h>

namespace connect4PicknPlace
{
    ros::ServiceClient client_picknplace;
    ros::ServiceClient client_move;

    float pickX = 0.341;
    float pickY = 0.351;
    float pickZ = 0.05;

    float dropXBasic = 0.356 + 0.031 * 3;
    float dropYBasic = -0.338;
    float dropZ = 0.23;

    float pickXOffset = 0;
    float pickYOffset = 0;
    float pickZOffset = 0;

    float dropXOffset = 0;
    float dropYOffset = -0.09;
    float dropZOffset = 0;

    float pickAngleRadians = 0;
    float dropAngleRadians = 0;

    std::tuple<float, float> getDropPos(int8_t dropIn)
    {
        float distance = 0.031;
        int toDrop = dropIn;
        toDrop -= 3;
        float xDir = std::cos(dropAngleRadians);
        float yDir = std::sin(dropAngleRadians);
        return {dropXBasic + xDir * distance * toDrop, dropYBasic + yDir * distance * toDrop};
    }

    bool callPicknPlaceService(float x, float y, float z, float a, float b, float c,
                               float x2, float y2, float z2, float a2, float b2, float c2)
    {
        ROS_INFO("Starting PickPlace");

        picknplace::PickNPlace srv;
        srv.request.blockX = x;
        srv.request.blockY = y;
        srv.request.blockZ = z;
        srv.request.blockA = a;
        srv.request.blockB = b;
        srv.request.blockC = c;
        srv.request.dropX = x2;
        srv.request.dropY = y2;
        srv.request.dropZ = z2;
        srv.request.dropA = a2;
        srv.request.dropB = b2;
        srv.request.dropC = c2;

        if (!client_picknplace.call(srv))
        {
            ROS_ERROR("Couldn't finish move");
            return false;
        }
        ROS_INFO("Finished PickPlace");
        return true;
    }

    bool callMoveService(float x, float y, float z, float a, float b, float c)
    {
        franka_ros_example::frankX_move srv;
        srv.request.x = x;
        srv.request.y = y;
        srv.request.z = z;
        srv.request.a = a;
        srv.request.b = b;
        srv.request.c = c;

        srv.request.speed = 0.1;
        srv.request.elbow = 0;
        srv.request.force_limit = 5;
        if (!client_move.call(srv))
        {
            ROS_ERROR("Couldn't finish move");
            return false;
        }
        ROS_INFO("Finished moving");
        return true;
    }

    bool moveChip(connect4_picknplace::connect4_picknplaceRequest &req, connect4_picknplace::connect4_picknplaceResponse &res)
    {
        ROS_INFO("Starting PickPlace");
        int8_t moveTo = req.dropIn;

        auto [dropX, dropY] = getDropPos(moveTo);

        return callPicknPlaceService(
            pickX + pickXOffset, pickY + pickYOffset, pickZ + pickZOffset,
            pickAngleRadians, 0, 0,
            dropX + dropXOffset, dropY + dropYOffset, dropZ + dropZOffset,
            dropAngleRadians, 0, 0);
    }

    bool movePickupToBoard(picknplace::Empty::Request &req, picknplace::Empty::Response &res)
    {
        auto [dropX, dropY] = getDropPos(3);
        return callPicknPlaceService(
            pickX + pickXOffset + 0.125, pickY + pickYOffset, pickZ + pickZOffset - 0.01,
            pickAngleRadians, 0, 0,
            dropX + dropXOffset, dropY + dropYOffset, dropZ + dropZOffset + 0.01,
            dropAngleRadians, 0, 0);
    }

    bool movePickupBack(picknplace::Empty::Request &req, picknplace::Empty::Response &res)
    {
        auto [dropX, dropY] = getDropPos(3);
        return callPicknPlaceService(
            dropX + dropXOffset, dropY + dropYOffset, dropZ + dropZOffset,
            dropAngleRadians, 0, 0,
            pickX + pickXOffset + 0.125, pickY + pickYOffset, pickZ + pickZOffset + 0.001,
            pickAngleRadians, 0, 0);
    }

    bool moveBoard(connect4_picknplace::singleCoords::Request &req, connect4_picknplace::singleCoords::Response &res)
    {
        auto [dropX, dropY] = getDropPos(3);
        bool result = callPicknPlaceService(
            dropX + dropXOffset, dropY + dropYOffset, dropZ + dropZOffset - 0.05,
            dropAngleRadians, 0, 0,
            dropX + dropXOffset + req.dropoffX, dropY + dropYOffset + req.dropoffY, dropZ + dropZOffset + req.dropoffZ - 0.047,
            dropAngleRadians + req.angleDiff, 0, 0);

        dropXOffset += req.dropoffX;
        dropYOffset += req.dropoffY;
        dropZOffset += req.dropoffZ;
        dropAngleRadians += req.angleDiff;
        return result;
    }

    bool moveCamera(picknplace::Empty::Request &req, picknplace::Empty::Response &res)
    {
        ROS_INFO("Moving to Camera Position");
        return callMoveService(0.5 + dropXOffset, -0.1 + dropYOffset, 0.15 + dropZOffset, 1.57, 1.57, 0);
    }

    bool testDropoff(connect4_picknplace::connect4_picknplaceRequest &req, connect4_picknplace::connect4_picknplaceResponse &res)
    {
        auto [dropX, dropY] = getDropPos(req.dropIn);
        return callMoveService(
            dropX + dropXOffset, dropY + dropYOffset, dropZ + dropZOffset,
            dropAngleRadians, 0, 0);
    }

    bool testPickup(picknplace::Empty::Request &req, picknplace::Empty::Response &res)
    {
        return callMoveService(
            pickX + pickXOffset, pickY + pickYOffset, pickZ + pickZOffset,
            pickAngleRadians, 0, 0);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "connect4_picknplace");

    // Load offsets from argv
    if (argc >= 2)
        connect4PicknPlace::pickXOffset = atof(argv[1]);
    if (argc >= 3)
        connect4PicknPlace::pickYOffset = atof(argv[2]);
    if (argc >= 4)
        connect4PicknPlace::pickZOffset = atof(argv[3]);
    if (argc >= 5)
        connect4PicknPlace::dropXOffset = atof(argv[4]);
    if (argc >= 6)
        connect4PicknPlace::dropYOffset = atof(argv[5]);
    if (argc >= 7)
        connect4PicknPlace::dropZOffset = atof(argv[6]);
    if (argc >= 8)
        connect4PicknPlace::pickAngleRadians = atof(argv[7]);
    if (argc >= 9)
        connect4PicknPlace::dropAngleRadians = atof(argv[8]);

    std::cout << "Pick Offsets: " << connect4PicknPlace::pickXOffset << "," << connect4PicknPlace::pickYOffset << "," << connect4PicknPlace::pickZOffset << std::endl;
    std::cout << "Drop Offsets: " << connect4PicknPlace::dropXOffset << "," << connect4PicknPlace::dropYOffset << "," << connect4PicknPlace::dropZOffset << std::endl;
    std::cout << "Rotations in Radians: " << connect4PicknPlace::pickAngleRadians << "," << connect4PicknPlace::dropAngleRadians << std::endl;

    ros::NodeHandle n;
    connect4PicknPlace::client_picknplace = n.serviceClient<picknplace::PickNPlace>("/picknplace");
    connect4PicknPlace::client_move = n.serviceClient<franka_ros_example::frankX_move>("/frankX_service_node/frankX_move");
    ros::ServiceServer picknPlace = n.advertiseService("moveChip", connect4PicknPlace::moveChip);
    ros::ServiceServer cameraMover = n.advertiseService("moveCameraToPicture", connect4PicknPlace::moveCamera);
    ros::ServiceServer testDropoff = n.advertiseService("testDropoff", connect4PicknPlace::testDropoff);
    ros::ServiceServer testPickup = n.advertiseService("testPickup", connect4PicknPlace::testPickup);
    ros::ServiceServer cheatMovePickupToBoard = n.advertiseService("cheatMovePickupToBoard", connect4PicknPlace::movePickupToBoard);
    ros::ServiceServer cheatMovePickupBack = n.advertiseService("cheatMovePickupBack", connect4PicknPlace::movePickupBack);
    ros::ServiceServer cheatMoveBoard = n.advertiseService("cheatMoveBoard", connect4PicknPlace::moveBoard);

    ROS_INFO("Ready");
    ros::spin();
    return 0;
}