import numpy as np
from .Minimax import minimaxWrapper, possibleMoves
from .alphazero import AlphaPlay
from .Evaluates import *
import random

playerTypes = ['hum', 'rand', 'minmaxLin', 'minmaxSq', 'minmaxCu', 'alphazero']

play = AlphaPlay()

def getNextMove(state, playerType, depth):
    boardSize = state.board.shape
    if playerType == 'hum':
        toPrint = str(np.linspace(0, boardSize[1] - 1, num = boardSize[1]))
        toPrint = np.char.replace(toPrint, ".", "")
        toPrint = np.char.replace(toPrint, "[", "")
        toPrint = np.char.replace(toPrint, "]", "")
        print(toPrint)
        print("Player " + ("🔵" if state.nextPlayer == -1 else "🔴") + "'s turn:")
        inp = None
        while inp == None:
            try:
                inp = int(input())
            except Exception:
                inp = None
                print("Invalid Input")
                continue

            if(inp < 0 or inp >= boardSize[1]):
                inp = None
                print("Invalid Number")
                continue
        return inp
    elif playerType == 'minmaxLin':
        return minimaxWrapper(state, depth, evaluateRowCountLinear)
    elif playerType == 'minmaxSq':
        return minimaxWrapper(state, depth, evaluateRowCountSquare)
    elif playerType == 'minmaxCu':
        return minimaxWrapper(state, depth, evaluateRowCountCube)
    elif playerType == 'alphazero':
        if state.nextPlayer == 1:
            return play.alphamove(np.flip(state.board, 0), "play")
        else:
            return play.alphamove(np.flip(-state.board, 0), "play")
    elif playerType == 'ran':
        moves = possibleMoves(state)
        random.shuffle(moves)
        return moves[0]
    else:
        print("Playertype " + str(playerType) + " is not known")
        return None