import rospy
import numpy as np
from connect4solver.BoardState import BoardState
from connect4solver.moveMaker import getNextMove, playerTypes
import sys
from connect4solver.srv import querySolver, querySolverResponse

playertype = "None"
depth = 5

def handleRequest(req):
    grid = np.reshape(req.board, ((6, 7)))
    grid = np.flip(grid, axis=0)
    print(grid)
    board = BoardState(grid, req.nextPlayer)
    print("Recieved Board: ")
    board.printBoard()

    win = board.isWin()
    if win == None:
        move = getNextMove(board, playertype, depth)
        print("Result: " + str(move))

        win = board.makeMove(move).isWin()

        print(win)
        return querySolverResponse(move, win, False)
    else:
        print("Finished game")
        print(win)
        return querySolverResponse(-1, win, True)


def main():
    if len(sys.argv) < 2:
       print("Please give what type of players should be used: " + str(playerTypes))
       return
    
    global playertype
    playertype = sys.argv[1]

    global depth
    if len(sys.argv) > 2:
        depth = float(sys.argv[2])
        print("Search depth is " + str(depth))
    else:
        print("No depth given. Using depth of " + str(depth))

    rospy.init_node('c4_solver')
    s = rospy.Service('c4_solver_service', querySolver, handleRequest)
    print("Ready!")
    rospy.spin()

if __name__ == '__main__':
    main()