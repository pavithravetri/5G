# =======================
# A* PATHFINDING BENCHMARK
# SINGLE FILE VERSION
# =======================

import math
import heapq
import time

# -----------------------
# Cell Types
# -----------------------
class CellType:
    START = "S"
    GOAL = "G"
    HIGH = "H"
    LOW = "L"
    WALL = "W"
    OPEN = "O"

# -----------------------
# GRID CLASS
# -----------------------
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
        if t == CellType.HIGH: return self.w_high
        if t == CellType.LOW: return self.w_low
        if t == CellType.WALL: return math.inf
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

# -----------------------
# Heuristics
# -----------------------
def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def euclidean(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def custom(a, b, terrain_cost):
    base = manhattan(a, b)
    penalty = 0.1 * terrain_cost
    return base + penalty

# -----------------------
# A* SEARCH
# -----------------------
def astar(grid, heuristic="manhattan"):
    start = grid.start
    goal = grid.goal

    def h(n):
        if heuristic == "manhattan": return manhattan(n, goal)
        if heuristic == "euclidean": return euclidean(n, goal)
        if heuristic == "custom":
            r, c = n
            return custom(n, goal, grid.cost(r, c))
        return 0

    open_set = []
    heapq.heappush(open_set, (h(start), 0, start))

    came_from = {}
    g_score = {start: 0}

    while open_set:
        f, g, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, start, goal), g_score[goal]

        r, c = current
        for nr, nc in grid.neighbors(r, c):
            new_g = g_score[current] + grid.cost(nr, nc)

            if (nr, nc) not in g_score or new_g < g_score[(nr, nc)]:
                g_score[(nr, nc)] = new_g
                came_from[(nr, nc)] = current
                heapq.heappush(open_set, (new_g + h((nr, nc)), new_g, (nr, nc)))

    return None, float("inf")


def reconstruct_path(came_from, start, goal):
    node = goal
    path = [node]
    while node != start:
        node = came_from[node]
        path.append(node)
    path.reverse()
    return path

# -----------------------
# BENCHMARKS
# -----------------------
MAPS = [
    [
        list("SOOOOOOOO"),
        list("OHHHHOHHO"),
        list("OOOLOOOHO"),
        list("OOLLLOOHO"),
        list("OOOLOOOOG")
    ],
    [
        list("SOHOOOHOG"),
        list("OHHHHOHHO"),
        list("OOOLOOOHO"),
        list("OOLLLOOHO"),
        list("OOOLOOOOO")
    ],
    [
        list("SOOOOOOOO"),
        list("OHHHHHHHO"),
        list("OLLLLLLHO"),
        list("OHHHHHHHO"),
        list("OOOOOOOOG")
    ],
    [
        list("SOOOLLLLG"),
        list("OHHHHHOOO"),
        list("OLOLOLOLO"),
        list("OHHHHHOOO"),
        list("OOOOOOOOO")
    ],
    [
        list("SOOOOHOOO"),
        list("OHHHHOHHO"),
        list("OOOOLOOHO"),
        list("OOOLLLOHO"),
        list("OOOLOOOOG")
    ]
]

HEURISTICS = ["manhattan", "euclidean", "custom"]

def run_benchmarks():
    print("Map | Heuristic | Path Cost | Time (ms)")
    print("----------------------------------------")

    for i, m in enumerate(MAPS):
        grid = Grid(m)
        for h in HEURISTICS:
            t1 = time.time()
            path, cost = astar(grid, heuristic=h)
            t2 = time.time()
            ms = (t2 - t1) * 1000
            print(f"{i+1}   | {h:<9} | {cost:<9} | {ms:.3f}")

# -----------------------
# MAIN EXECUTION
# -----------------------
if __name__ == "__main__":
    run_benchmarks()
