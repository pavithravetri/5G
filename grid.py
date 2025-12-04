# grid.py
import math

class CellType:
    START = "S"
    GOAL = "G"
    HIGH = "H"
    LOW = "L"
    WALL = "W"
    OPEN = "O"

class Grid:
    def __init__(self, grid, w_low=1, w_high=5):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.w_low = w_low
        self.w_high = w_high
        self.start = self.find(CellType.START)
        self.goal = self.find(CellType.GOAL)

    def find(self, cell_type):
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == cell_type:
                    return (r, c)
        return None

    def cost(self, r, c):
        t = self.grid[r][c]
        if t == CellType.HIGH:
            return self.w_high
        if t == CellType.LOW:
            return self.w_low
        if t == CellType.WALL:
            return math.inf
        return 1

    def in_bounds(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols

    def neighbors(self, r, c):
        moves = [(1,0), (-1,0), (0,1), (0,-1)]
        result = []
        for dr, dc in moves:
            nr, nc = r + dr, c + dc
            if self.in_bounds(nr, nc) and self.grid[nr][nc] != CellType.WALL:
                result.append((nr, nc))
        return result
