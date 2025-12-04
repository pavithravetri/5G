# astar.py
import heapq
from heuristics import manhattan, euclidean, custom

def astar(grid, heuristic="manhattan"):
    start = grid.start
    goal = grid.goal

    def h(n):
        if heuristic == "manhattan":
            return manhattan(n, goal)
        if heuristic == "euclidean":
            return euclidean(n, goal)
        if heuristic == "custom":
            r, c = n
            return custom(n, goal, grid.cost(r, c))
        return 0

    open_set = []
    heapq.heappush(open_set, (0 + h(start), 0, start))  # (f, g, node)

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

    return None, float('inf')  # No path found


def reconstruct_path(came_from, start, goal):
    node = goal
    path = [node]
    while node != start:
        node = came_from[node]
        path.append(node)
    path.reverse()
    return path
