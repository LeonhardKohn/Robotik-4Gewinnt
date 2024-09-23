#include "ros/ros.h"
#include <connect4_picknplace/connect4_picknplace.h>
#include <connect4_picknplace/singleCoords.h>
#include <connect4solver/querySolver.h>
#include <object_detection/DetectGrid.h>
#include <picknplace/Empty.h>
#include <stdio.h>
#include <string>
#include <unistd.h>

namespace connect4Player
{
    int8_t isStartPlayer = 0;
    bool colorIsRed = true;
    int current_turn = 0;
    int cheatType = 0;

    bool isCheatActive = false;

    ros::ServiceClient client_picknplace;
    ros::ServiceClient client_connect4Solver;
    ros::ServiceClient client_detectGrid;
    ros::ServiceClient client_moveToPicturePos;
    ros::ServiceClient client_moveHome;
    ros::ServiceClient client_cheatBoardMove;
    ros::ServiceClient client_cheatPickupPlace;
    ros::ServiceClient client_cheatPickupReturn;

    std::vector<int8_t> detectBoard(int current_turn)
    {
        object_detection::DetectGrid srv;
        srv.request.current_turn = current_turn;
        srv.request.isStartPlayer = isStartPlayer;
        if (colorIsRed)
        {
            srv.request.color = 1; // red is 1, blue is 0
        }
        else
        {
            srv.request.color = 0;
        }
        if (!client_detectGrid.call(srv))
        {
            ROS_ERROR("Couldn't call detectGrid");
            return std::vector<int8_t>();
        }
        while (srv.response.detected_grid.size() == 0)
        {
            ROS_ERROR("Camera Issues!!!! Waiting for button press to continue");
            ROS_INFO("Press x or c to cancel");
            std::string input;
            std::getline(std::cin, input);
            if (input == "x" || input == "c" || input == "C" || input == "X")
            {
                ROS_INFO("Terminating via button press. Goodbye");
                return std::vector<int8_t>();
            }
            client_detectGrid.call(srv);
        }
        std::vector<int8_t> result = srv.response.detected_grid;
        return result;
    }

    int8_t querySolver(std::vector<int8_t> board)
    {
        connect4solver::querySolver srv;
        srv.request.board = board;
        srv.request.nextPlayer = isStartPlayer;
        if (!client_connect4Solver.call(srv))
            return 125; // Error
        int8_t isWinFor = srv.response.isWinFor;
        if ((isStartPlayer == 1) != colorIsRed)
            isWinFor *= -1;
        if (srv.response.gameFinished && srv.response.result == -1)
        {
            if (isWinFor == 0)
            {
                ROS_WARN("It seems we're evenly matched.");
                return 122; // Gleichstand
            }
            else
            {
                if (isWinFor == isStartPlayer)
                {
                    ROS_WARN("I have won!");
                    return 120; // Win for you
                }
                else
                {
                    ROS_WARN("You win this one...");
                    return 121; // Win for other player
                }
            }
        }
        return srv.response.result;
    }

    bool cheatBoardPos(bool undo)
    {
        ROS_INFO("Cheating!");
        connect4_picknplace::singleCoords srv;
        srv.request.dropoffX = 0;
        srv.request.dropoffY = 0;
        srv.request.dropoffZ = 0;
        srv.request.angleDiff = 0.2;
        if (undo)
            srv.request.angleDiff *= -1;
        return client_cheatBoardMove.call(srv);
    }

    bool cheatPickupOnBoard(bool undo)
    {
        ROS_INFO("Cheating!");
        picknplace::Empty srv;
        if (!undo)
            return client_cheatPickupPlace.call(srv);
        else
            return client_cheatPickupReturn.call(srv);
    }

    bool executePicknPlace(int8_t move)
    {
        connect4_picknplace::connect4_picknplace srv;
        srv.request.dropIn = move;
        return client_picknplace.call(srv);
    }

    bool doOneMove(int current_turn)
    {
        if (isCheatActive)
        {
            switch (cheatType)
            {
            case 1:
                cheatPickupOnBoard(true);
                isCheatActive = false;
                break;
            case 2:
                cheatBoardPos(true);
                isCheatActive = false;
                break;

            default:
                break;
            }
        }

        // Move to make an image
        ROS_INFO("Moving to camera position");
        {
            picknplace::Empty srv;
            if (!client_moveToPicturePos.call(srv))
            {
                ROS_ERROR("Couldn't move to camera position");
                return false;
            }
        }
        // Wait for image to be taken
        ROS_INFO("Waiting at camera position");
        sleep(2);
        // Load Boardstate via Camera
        ROS_INFO("Taking picture");
        std::vector<int8_t> board = detectBoard(current_turn);
        if (board.size() < 42)
        {
            return false;
        }
        // Reverse board numbers to match startplayer and color
        if ((isStartPlayer == 1) != colorIsRed)
        {
            for (int i = 0; i < board.size(); i++)
            {
                board[i] *= -1;
            }
        }

        // Move to home
        ROS_INFO("Moving to home position");
        {
            picknplace::Empty srv;
            if (!client_moveHome.call(srv))
            {
                ROS_ERROR("Couldn't move to home position");
                return false;
            }
        }
        // Optional: Check if move from other party was valid
        // TODO

        // Query the connect4Solver
        ROS_INFO("Finding good move");
        int8_t move = querySolver(board);
        switch (move)
        {
        case 120:
        case 121:
        case 122:
            ROS_INFO("Game finshed");
            return false;
        case 125:
            ROS_ERROR("Failed to query Solver!");
            return false;
        default:
            break;
        }
        // Correct Indexing
        int moveN = move;
        std::cout << moveN << std::endl;
        move = 6 - move;
        std::cout << moveN << std::endl;

        // Picknplace Stone
        ROS_INFO("PicknPlacing chip");
        if (!executePicknPlace(move))
        {
            ROS_ERROR("Failed to execute Move");
            return false;
        }

        if (!isCheatActive)
        {
            switch (cheatType)
            {
            case 1:
                cheatPickupOnBoard(false);
                isCheatActive = true;
                break;
            case 2:
                cheatBoardPos(false);
                isCheatActive = true;
                break;

            default:
                break;
            }
        }

        return true;
    }

    void playGame()
    {
        bool playing = true;
        int movesDone = 0;
        while (playing && movesDone < 100)
        {
            ROS_INFO("Waiting for button press...");
            std::string input;
            std::getline(std::cin, input);
            if (input == "x" || input == "c" || input == "C" || input == "X")
            {
                ROS_INFO("Terminating via button press. Goodbye");
                break;
            }
            playing = doOneMove(movesDone);
            movesDone++;
            ROS_INFO("Finished Move");
        }
        ROS_INFO("Terminating!");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "playConnect4Main");

    if (argc < 3)
    {
        ROS_ERROR("Requires the start arguments StartPlayer (\"you\" | \"other\") and PlayColor (\"red\" | \"blue\")");
        return 1;
    }

    std::string startPlayer = argv[1];
    std::string color = argv[2];

    if ((startPlayer != "you" && startPlayer != "other") || (color != "red" && color != "blue"))
    {
        std::cout << startPlayer << std::endl;
        std::cout << color << std::endl;
        ROS_ERROR("Requires the start arguments StartPlayer (\"you\" | \"other\") and PlayColor (\"red\" | \"blue\")");
        return 1;
    }

    if (startPlayer == "you")
        connect4Player::isStartPlayer = 1;
    else
        connect4Player::isStartPlayer = -1;

    connect4Player::colorIsRed = color == "red";

    if (argc > 3)
    {
        std::string cheatType = argv[3];
        if (cheatType == "Pickup")
        {
            connect4Player::cheatType = 1;
            std::cout << "Using Pickup on top of Board Cheat" << std::endl;
        }
        else if (cheatType == "Board")
        {
            connect4Player::cheatType = 2;
            std::cout << "Using slightly rotate Board Cheat" << std::endl;
        }
        else
        {
            connect4Player::cheatType = 0;
            std::cout << "Using no cheats" << std::endl;
        }
    }
    else
    {
        connect4Player::cheatType = 0;
        std::cout << "Using no cheats" << std::endl;
    }

    ros::NodeHandle n;
    connect4Player::client_picknplace = n.serviceClient<connect4_picknplace::connect4_picknplace>("/moveChip");
    connect4Player::client_connect4Solver = n.serviceClient<connect4solver::querySolver>("/c4_solver_service");
    connect4Player::client_detectGrid = n.serviceClient<object_detection::DetectGrid>("/detect_grid");
    connect4Player::client_moveToPicturePos = n.serviceClient<picknplace::Empty>("/moveCameraToPicture");
    connect4Player::client_moveHome = n.serviceClient<picknplace::Empty>("/movehome");
    connect4Player::client_cheatBoardMove = n.serviceClient<connect4_picknplace::singleCoords>("/cheatMoveBoard");
    connect4Player::client_cheatPickupPlace = n.serviceClient<picknplace::Empty>("/cheatMovePickupToBoard");
    connect4Player::client_cheatPickupReturn = n.serviceClient<picknplace::Empty>("/cheatMovePickupBack");

    // Cheat Rotation with 0.15 is enough!

    ROS_INFO("Ready");
    ROS_WARN("THIS CANNOT BE TERMINATED BY CTRL+C");
    ROS_WARN("Type x or c to terminate this program");
    connect4Player::playGame();
    ROS_INFO("Finished Execution");

    return 0;
}