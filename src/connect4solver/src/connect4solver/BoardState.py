import numpy as np
from scipy.signal import convolve2d

class BoardState:
    
    # board is a numpy array containing only the numbers -1, 0, 1
    # nextPlayer is either -1 or 1
    def __init__(self, board, nextPlayer):
        if(nextPlayer != -1 and nextPlayer != 1):
            raise Exception("nextPlayer has to be either -1 or 1, not " + str(nextPlayer))

        self.board = np.array(board) #Copy board
        self.nextPlayer = nextPlayer


    def printBoard(self):
        toPrint = str(np.flip(self.board, axis=0))
        toPrint = np.char.replace(toPrint, ".", "")
        toPrint = np.char.replace(toPrint, "[", "")
        toPrint = np.char.replace(toPrint, "]", "")
        toPrint = np.char.replace(toPrint, " ", "")
        toPrint = np.char.replace(toPrint, "-1", "🔵")
        toPrint = np.char.replace(toPrint, "1", "🔴")
        toPrint = np.char.replace(toPrint, "0", "⚪")

        print(toPrint)

    def getLowestFreeTile(self, column):
        col = self.board[:, column]
        zeroes = np.where(col == 0)[0]
        return zeroes[0]

    def canDropInColumn(self, column):
        return self.board[5, column] == 0

    def makeMove(self, column):
        if(not self.canDropInColumn(column)):
            raise Exception("Column already full!")
        
        newBoard = np.array(self.board)
        l = self.getLowestFreeTile(column)

        newBoard[l, column] = self.nextPlayer
        return BoardState(newBoard, self.nextPlayer * -1)
    
    def isWin(self):
        kernel = np.ones((1, 4), dtype=int)
        
        # Horizontal and vertical checks
        horizontal_check = convolve2d(self.board, kernel, mode='valid')
        vertical_check = convolve2d(self.board, kernel.T, mode='valid')

        # Diagonal checks
        diagonal_kernel = np.eye(4, dtype=int)
        main_diagonal_check = convolve2d(self.board, diagonal_kernel, mode='valid')
        anti_diagonal_check = convolve2d(self.board, np.fliplr(diagonal_kernel), mode='valid')
        
        # Check for winner
        if any(cond.any() for cond in [horizontal_check == 4, vertical_check == 4, main_diagonal_check == 4, anti_diagonal_check == 4]):
            return 1
        elif any(cond.any() for cond in [horizontal_check == -4, vertical_check == -4, main_diagonal_check == -4, anti_diagonal_check == -4]):
            return -1
        
        if(np.where(self.board == 0)[0].shape[0] == 0):
            return 0
        return None
    
    def invertBoard(self):
        newBoard = np.flip(self.board, axis=1)
        return BoardState(newBoard, self.nextPlayer)

