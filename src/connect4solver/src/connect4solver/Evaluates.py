from scipy.signal import convolve2d
import numpy as np

def evaluateRowCountLinear(state):
    return evaluateRowCount(state, 1)

def evaluateRowCountSquare(state):
    return evaluateRowCount(state, 2)

def evaluateRowCountCube(state):
    return evaluateRowCount(state, 3)

def evaluateRowCount(state, power):
    val = state.isWin()
    if not val == None:
        return val * 1000000000

    board_0 = np.array(state.board)
    board_0[board_0 < 0] = 0
    board_1 = np.array(state.board)
    board_1[board_1 > 0] = 0
        
    kernel = np.ones((1, 4), dtype=int)
    # Horizontal and vertical checks
    horizontal_check_0 = convolve2d(board_0, kernel, mode='valid')
    horizontal_check_1 = convolve2d(board_1, kernel, mode='valid')
    vertical_check_0 = convolve2d(board_0, kernel.T, mode='valid')
    vertical_check_1 = convolve2d(board_1, kernel.T, mode='valid')

    mask_0 = horizontal_check_0 != 0
    mask_1 = horizontal_check_1 != 0
    horizontal_check_0[mask_1] = 0
    horizontal_check_1[mask_0] = 0

    horizontal_check = horizontal_check_0 + horizontal_check_1
    horizontal_points = np.ones(horizontal_check.shape)
    for i in range(power):
        horizontal_points = horizontal_points * horizontal_check
    if power % 2 == 0:
        horizontal_points = horizontal_points * np.sign(horizontal_check)

    mask_0 = vertical_check_0 != 0
    mask_1 = vertical_check_1 != 0
    vertical_check_0[mask_1] = 0
    vertical_check_1[mask_0] = 0

    vertical_check = vertical_check_0 + vertical_check_1
    vertical_points = np.ones(vertical_check.shape)
    for i in range(power):
        vertical_points = vertical_points * vertical_check
    if power % 2 == 0:
        vertical_points = vertical_points * np.sign(vertical_check)

    # Diagonal checks
    diagonal_kernel = np.eye(4, dtype=int)
    main_diagonal_check_0 = convolve2d(board_0, diagonal_kernel, mode='valid')
    main_diagonal_check_1 = convolve2d(board_1, diagonal_kernel, mode='valid')
    anti_diagonal_check_0 = convolve2d(board_0, np.fliplr(diagonal_kernel), mode='valid')
    anti_diagonal_check_1 = convolve2d(board_1, np.fliplr(diagonal_kernel), mode='valid')
    
    mask_0 = main_diagonal_check_0 != 0
    mask_1 = main_diagonal_check_1 != 0
    main_diagonal_check_0[mask_1] = 0
    main_diagonal_check_1[mask_0] = 0

    main_diagonal_check = main_diagonal_check_0 + main_diagonal_check_1
    main_diagonal_points = np.ones(main_diagonal_check.shape)
    for i in range(power):
        main_diagonal_points = main_diagonal_points * main_diagonal_check
    if power % 2 == 0:
        main_diagonal_points = main_diagonal_points * np.sign(main_diagonal_check)

    mask_0 = anti_diagonal_check_0 != 0
    mask_1 = anti_diagonal_check_1 != 0
    anti_diagonal_check_0[mask_1] = 0
    anti_diagonal_check_1[mask_0] = 0

    anti_diagonal_check = anti_diagonal_check_0 + anti_diagonal_check_1
    anti_diagonal_points = np.ones(anti_diagonal_check.shape)
    for i in range(power):
        anti_diagonal_points = anti_diagonal_points * anti_diagonal_check
    if power % 2 == 0:
        anti_diagonal_points = anti_diagonal_points * np.sign(anti_diagonal_check)

    totalPoints = np.sum(horizontal_points) + np.sum(vertical_points) + np.sum(main_diagonal_points) + np.sum(anti_diagonal_points)
    return totalPoints