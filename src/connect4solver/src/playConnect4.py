import numpy as np
import sys
import time
from connect4solver.alphazero import AlphaPlay
from connect4solver.moveMaker import getNextMove, playerTypes
from connect4solver.BoardState import BoardState

def main():
    boardSize = (6, 7)
    if len(sys.argv) < 3:
       print("Please give what type of players should be used: " + str(playerTypes))
       return

    playerOne = sys.argv[1] 
    playerTwo = sys.argv[2] 

    depth = 5
    if len(sys.argv) > 3:
        depth = float(sys.argv[3])
        print("Search depth is " + str(depth))
    else:
        print("No depth given. Using depth of " + str(depth))


    state = BoardState(np.zeros(boardSize), 1)

    startTime = time.time()

    while True:
        state.printBoard()

        if state.nextPlayer == 1:
            nextMove = getNextMove(state, playerOne, depth)
        else:
            nextMove = getNextMove(state, playerTwo, depth)

        state = state.makeMove(nextMove)

        isWin = state.isWin()
        if(isWin == -1 or isWin == 1):
            state.printBoard()
            print("Player " + ("🔵" if state.nextPlayer == 1 else "🔴") + " has won")
            break
        if(isWin == 0):
            state.printBoard()
            print("Draw")
            break
    
    print("Total Execution Time: " + str(time.time() - startTime))

if __name__ == '__main__':
    main()