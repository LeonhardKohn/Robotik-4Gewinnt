# connect4

## picknplace
This package adds the pick and place functionality from the second worksheet. This functionality is provided as a service, that can be started with the command:

`rosrun picknplace picknplaceService`

It defines the services:
- "/picknplaceBlock" which takes the pickup coordinates, pickup rotation, dropoff coordinates and dropoff rotation and performs the entire picknplace procedure.
- "/movehome": Moves the robot into the home position.

## connect4solver

This package offers services that given a board state will try to find the best possible move.

This can be started using the command:

`rosrun connect4solver solverService.py minmaxSq`

The "minmaxSq" determines that the player uses the minimax algorithm to predict moves using the squared version of the evaluate function. See the bottom of the readme for other player types.

It defines the service "/c4_solver_service" which given a boardstate will calculate the best possible move with the given algorithm.

### Play in Console
In order to play connect4 in the console, you can use the following command

`rosrun connect4solver playConnect4.py hum hum`

Changing the search tree depth of the bot can be done as an additinal argument (default 3). This depth is given in full turns, not half turns and can be set to a number ending in .5

`rosrun connect4solver playConnect4.py minmaxSq minmaxSq 2`

### All Player Types:
- hum (Asks for next move via console)
- minmaxLin (Uses Minimax to determine best move with linear evaluate)
- minmaxSq (Uses Minimax to determine best move with squared evaluate)
- minmaxCu (Uses Minimax to determine best move with cubed evaluate)
- ran (Chooses Random Moves)

## connect4_picknplace

This package provides functionality for all tasks relatet to the positioning of the board.
Currently it only has the version static_picknplace which uses static coordinates.

`rosrun connect4_picknplace static_picknplace`

This program can take 8 additional arguments. These move the pickup and dropoff positions, allowing the position the be adjusted quickly. As the board is static all dropoff positions are translated together. Testing whether the chosen offsets are correct can be done via the services "/testDropoff" and "/testPickup" (see below). The order of arguments is:

- Pickup X Offset
- Pickup Y Offset
- Pickup Z Offset
- Dropoff X Offset
- Dropoff Y Offset
- Dropoff Z Offset
- Pickup Rotation in Radians
- Dropoff Rotation in Radians

The program defines the following services:

- "/moveChip": Moves a chip from the pickup to the given dropoff column
- "/moveCameraToPicture": Moves the robot into a good position to take a picture
- "/testDropoff": (For Debugging) Moves Robot to the Dropoff column given by the service
- "/testPickup": (For Debugging) Moves Robot to the Pickup Spot

This programm defines extra services used for cheating:

- "/cheatMovePickupToBoard": Moves the entire pickup tray on top of the board to block it.
- "/cheatMovePickupBack": Moves the pickup tray back
- "/cheatMoveBoard": Moves the board a given distance and rotation. Changed values are updated in this class

## object_detection

This package gives a service to read the boardstate via the camera.

`rosrun object_detection yolo_service_node.py`

It defines the service "/detect_grid" which takes a picture and detects the current board state.

This package has a dummy executable that returns a static board defined in code:

`rosrun object_detection dummy_service.py`

## playconnect4
This package is responsible for calling the other packages and actually playing the game. It can be started with the command:

`rosrun playconnect4 main you red`

The command requires two arguments:
- Start Player: Determines whether this robot is the start player. Allows values "you" and "other"
- Own Color: Determines the color this robot will have. Allows values "red" and "blue"

There is an optional third argument that determines what cheating method will be used. It allows the arguments:
- "Board"
- "Pickup"

## dummy_franka

This package provides a dummy implementation of the three frankX services. This allows testing the program without having a robot attached to them.

`rosrun dummy_franka dummy`

## Command Collection
### Main Commands

`rosrun picknplace picknplaceService`

`rosrun connect4solver solverService.py minmaxSq`

`rosrun connect4_picknplace static_picknplace`

`rosrun object_detection yolo_service_node.py`

`rosrun playconnect4 main you red`

### Real Robot Commands

`roslaunch franka_ros_example frankX_service.launch robot_ip:=...`

`roslaunch realsense2_camera rs_camera.launch`

### Simulation Commands
`roslaunch franka_ros_example frankX_service_sim.launch`

`roslaunch franka_gazebo panda_moveit.launch world:=$(rospack find franka_gazebo)/world/pickandplace.sdf controller:=joint_position_example_controller`