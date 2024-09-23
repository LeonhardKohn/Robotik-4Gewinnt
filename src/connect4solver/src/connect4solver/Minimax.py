from .BoardState import BoardState
import numpy as np
import random
import time

def possibleMoves(state):
    if state.isWin():
        return []
    allCols = list(range(state.board.shape[1]))
    return list(filter(lambda a: state.canDropInColumn(a), allCols))

lastMoves = []
def minimaxWrapper(state, depth, evaluateFunc):
    curTime = time.time()
    global lastMoves
    depth *= 2 # Convert half moves to full moves
    tptable = dict()
    (_, move) = minimax_ab(state, depth, -np.inf * state.nextPlayer, np.inf * state.nextPlayer, tptable, lastMoves, evaluateFunc)
    print(time.time() - curTime)
    lastMoves = move
    print(lastMoves)
    print(evaluateFunc(state.makeMove(move[0])))
    return move[0]

def minimax_ab(state, depthRemaining, alpha, beta, tptable, lastMoves, evaluateFunc):
    moves = possibleMoves(state)
    if depthRemaining <= 0 or len(moves) == 0:
        return (evaluateFunc(state) - 0.01 * depthRemaining * state.nextPlayer, [])
    
    #moves.sort(key=lambda a:abs(a - state.board.shape[1] / 2))

    #Sort by index in lastMoves, otherwise by distance to center
    #print(str(moves) + ":" + str(lastMoves))
    moves.sort(key=lambda x: lastMoves.index(x) if x in lastMoves else 100 * abs(x - state.board.shape[1] / 2))
    #print(moves)

    compare = (lambda a, b: a > b) if state.nextPlayer == 1 else (lambda a, b: a < b)
    
    saved = (-np.inf * state.nextPlayer, [])
    for move in moves:
        nextState = state.makeMove(move)
        lookup = str(nextState.board)
        if lookup in tptable:
            (val, m) = tptable[lookup]
        else:
            invLookup = str(nextState.invertBoard().board)
            if invLookup in tptable:
                (val, m) = tptable[invLookup]
            else:
                (val, m) = minimax_ab(nextState, depthRemaining - 1, beta, alpha, tptable, lastMoves, evaluateFunc)
                tptable[lookup] = (val, m)
        #if depthRemaining == 3:
        #    for i in range(int((3 - depthRemaining) * 2)):
        #        print("\t", end='')
        #    print(str(nextState.board))
        #    for i in range(int((3 - depthRemaining) * 2)):
        #        print("\t", end='')
        #    print(str(val) + ": " + str(move) +  "(" + str(m) + ")" + ": " + str(depthRemaining - 0.5))
        
        if compare(val, saved[0]):
            saved = (val, [move] + m)
            if compare(val, beta):
                break
            if compare(val, alpha):
                alpha = val
    
    return saved
