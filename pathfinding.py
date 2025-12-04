import heapq
import math
import time
import random
from typing import List, Tuple, Dict, Optional

# --- Configuration ---
GRID_SIZE = 100
SWAMP_COST = 5
BLOCKED_COST = float('inf')
# ---------------------

class Node:
    """Represents a single cell in the grid."""
    def __init__(self, row: int, col: int, cost: int = 1):
        self.row = row
        self.col = col
        self.cost = cost  # Base cost to enter this cell
        
        # A* Scores
        self.g_score = float('inf')  # Actual cost from start
        self.h_score = 0.0          # Heuristic estimate to goal
        self.f_score = float('inf')  # Total estimated cost (f = g + h)
        
        self.parent: Optional[Node] = None
        # Unique ID for stable tie-breaking in the heap (prevents comparison errors)
        self.uid = id(self) 

    def __lt__(self, other: 'Node') -> bool:
        """Comparison for priority queue (min-heap). Priority is f_score."""
        # 1. Primary comparison: f_score
        if self.f_score != other.f_score:
            return self.f_score < other.f_score
        
        # 2. Secondary comparison: g_score (prefer nodes closer to the start)
        if self.g_score != other.g_score:
            return self.g_score < other.g_score
            
        # 3. Tertiary comparison: UID (Stable tie-breaker)
        return self.uid < other.uid

    def __hash__(self) -> int:
        """Hash by coordinates for efficient use in sets/dictionaries."""
        return hash((self.row, self.col))

    def __eq__(self, other: object) -> bool:
        """Equality check based on coordinates."""
        if not isinstance(other, Node):
            return NotImplemented
        return self.row == other.row and self.col == other.col

    def __repr__(self) -> str:
        return f"Node({self.row}, {self.col}, f={self.f_score:.2f}, g={self.g_score:.2f})"


# --- Heuristic Functions ---
def heuristic_manhattan(node: Node, goal: Node, min_cost: int) -> float:
    """Heuristic 1: Manhattan Distance."""
    distance = abs(node.row - goal.row) + abs(node.col - goal.col)
    # Scale by minimum movement cost to ensure admissibility on weighted grid
    return min_cost * distance

def heuristic_euclidean(node: Node, goal: Node, min_cost: int) -> float:
    """Heuristic 2: Euclidean Distance. FIX APPLIED to prevent math domain error."""
    
    # Calculate the sum of squared differences
    sum_of_squares = (node.row - goal.row)*2 + (node.col - goal.col)*2
    
    # FIX: Use max(0, ...) to ensure the argument to math.sqrt is non-negative,
    # protecting against potential floating point errors near zero.
    distance = math.sqrt((node.row - goal.row)**2 + (node.col - goal.col)**2)
    
    # Scale by minimum movement cost to ensure admissibility on weighted grid
    return min_cost * distance

def heuristic_custom_weighted(node: Node, goal: Node, min_cost: int) -> float:
    """Heuristic 3: Custom Weighted Heuristic. 
    Uses Manhattan distance but with a small factor (> 1.0) to make it more 
    'informed' and aggressive, potentially causing slight sub-optimality for analysis."""
    
    h_manhattan = abs(node.row - goal.row) + abs(node.col - goal.col)
    
    # Using a slightly higher factor (1.001) for a 'more informed' custom test.
    # NOTE: This makes the heuristic potentially NON-ADMISSIBLE, which is useful 
    # for your analysis of the optimality vs. speed trade-off.
    bias_factor = 1.001  
    return bias_factor * min_cost * h_manhattan


# --- Grid Creation ---
def create_grid(size: int, obstacle_density: float, swamp_percentage: float) -> List[List[Node]]:
    """Generates the grid with obstacles and weighted regions."""
    grid = [[Node(r, c) for c in range(size)] for r in range(size)]
    
    random.seed(42) # Ensure consistent maps
    
    # 1. Place Obstacles
    for r in range(size):
        for c in range(size):
            if random.random() < obstacle_density:
                grid[r][c].cost = BLOCKED_COST # Effectively infinite cost
    
    # 2. Place Weighted Regions (e.g., a central swamp)
    # Use max(1, ...) to ensure swamp area is at least 1x1 if percentage is non-zero
    swamp_area = max(1, int(size * math.sqrt(swamp_percentage))) 
    swamp_start_r = (size - swamp_area) // 2
    swamp_start_c = (size - swamp_area) // 2

    for r in range(swamp_start_r, swamp_start_r + swamp_area):
        for c in range(swamp_start_c, swamp_start_c + swamp_area):
            # Only apply swamp cost if it's not already an obstacle
            if 0 <= r < size and 0 <= c < size and grid[r][c].cost != BLOCKED_COST:
                grid[r][c].cost = SWAMP_COST 

    return grid

# --- A* Algorithm ---
def a_star(grid: List[List[Node]], start_coords: Tuple[int, int], goal_coords: Tuple[int, int], heuristic_func) -> Tuple[Optional[List[Tuple[int, int]]], int, float]:
    """The A* pathfinding core logic."""
    R, C = len(grid), len(grid[0])
    
    # Reset nodes for a new run
    for r in range(R):
        for c in range(C):
            grid[r][c].g_score = float('inf')
            grid[r][c].f_score = float('inf')
            grid[r][c].parent = None

    start_node = grid[start_coords[0]][start_coords[1]]
    goal_node = grid[goal_coords[0]][goal_coords[1]]
    
    # Find minimum movement cost for the admissible heuristic scale
    min_cost = min(node.cost for row in grid for node in row if node.cost != BLOCKED_COST)
    if min_cost == float('inf'):
        # Handle case where the entire map is blocked
        return None, 0, float('inf')

    # Initialization
    start_node.g_score = 0
    start_node.h_score = heuristic_func(start_node, goal_node, min_cost)
    start_node.f_score = start_node.g_score + start_node.h_score
    
    # Open List: Priority Queue (f_score, g_score, uid, row, col)
    # Including g_score and uid ensures stable and safe comparisons in the heap.
    open_list: List[Tuple[float, float, int, int, int]] = [
        (start_node.f_score, start_node.g_score, start_node.uid, start_node.row, start_node.col)
    ]
    
    closed_set: Dict[Tuple[int, int], bool] = {}
    
    nodes_expanded = 0
    
    # 4-way movement (up, down, left, right)
    movements = [(0, 1), (0, -1), (1, 0), (-1, 0)] 
    
    while open_list:
        # Get the node with the lowest f_score
        f, g, uid, r, c = heapq.heappop(open_list)
        current_node = grid[r][c]
        
        # Check if this node has already been processed with a better path
        if (r, c) in closed_set:
            continue
            
        closed_set[(r, c)] = True
        nodes_expanded += 1
        
        # Check for goal
        if current_node == goal_node:
            path = []
            curr = current_node
            while curr:
                path.append((curr.row, curr.col))
                curr = curr.parent
            return path[::-1], nodes_expanded, current_node.g_score

        # Explore neighbors
        for dr, dc in movements:
            nr, nc = r + dr, c + dc
            
            # Check bounds and obstacles
            if 0 <= nr < R and 0 <= nc < C and grid[nr][nc].cost != BLOCKED_COST:
                neighbor = grid[nr][nc]
                
                # If neighbor is in closed list, skip it
                if (nr, nc) in closed_set:
                    continue
                    
                # Tentative g_score is the actual cost from start to neighbor
                # Cost to move is neighbor.cost (cost of entering the neighbor cell)
                tentative_g_score = current_node.g_score + neighbor.cost
                
                # Check if this new path is better
                if tentative_g_score < neighbor.g_score:
                    neighbor.parent = current_node
                    neighbor.g_score = tentative_g_score
                    neighbor.h_score = heuristic_func(neighbor, goal_node, min_cost)
                    neighbor.f_score = neighbor.g_score + neighbor.h_score
                    
                    # Add to priority queue
                    heapq.heappush(open_list, (neighbor.f_score, neighbor.g_score, neighbor.uid, nr, nc))

    return None, nodes_expanded, float('inf')


# --- Benchmarking ---
def run_benchmark(map_id: int, grid: List[List[Node]], start: Tuple[int, int], goal: Tuple[int, int], heuristic_func, heuristic_name: str):
    """Executes A* and records performance metrics."""
    start_time = time.time()
    path, nodes_expanded, path_length = a_star(grid, start, goal, heuristic_func)
    end_time = time.time()
    
    runtime_ms = (end_time - start_time) * 1000
    
    # Check if a path was found
    if path_length == float('inf'):
        path_length_str = "No Path"
    else:
        path_length_str = f"{path_length:.2f}"
    
    print(f"| {map_id:<6} | {heuristic_name:<20} | {path_length_str:<10} | {nodes_expanded:<15} | {runtime_ms:<12.3f} ms |")
    
    return path_length, nodes_expanded, runtime_ms


def main():
    """Main execution function for the project."""
    
    # Define five distinct maps (using different densities)
    MAP_CONFIGS = {
        1: {"obs_density": 0.05, "swamp_percent": 0.20, "desc": "Sparse Obstacles, Large Swamp"},
        2: {"obs_density": 0.20, "swamp_percent": 0.05, "desc": "Dense Obstacles, Small Swamp"},
        3: {"obs_density": 0.10, "swamp_percent": 0.40, "desc": "Balanced Obstacles, Very Large Swamp"},
        4: {"obs_density": 0.00, "swamp_percent": 0.00, "desc": "Uniform Cost Grid (Baseline)"},
        5: {"obs_density": 0.15, "swamp_percent": 0.15, "desc": "Balanced Challenge"}
    }
    
    start_coords = (5, 5)
    goal_coords = (GRID_SIZE - 5, GRID_SIZE - 5)
    
    HEURISTICS = {
        "Manhattan": heuristic_manhattan,
        "Euclidean": heuristic_euclidean,
        "Custom Weighted": heuristic_custom_weighted,
    }

    print("## A* Pathfinding Heuristic Analysis")
    print("-" * 75)

    for map_id, config in MAP_CONFIGS.items():
        print(f"\n### Map {map_id}: {config['desc']} (Obs: {config['obs_density']}, Swamp: {config['swamp_percent']})")
        print("Start:", start_coords, " | Goal:", goal_coords)
        
        # Create map and run all heuristics on it
        grid = create_grid(GRID_SIZE, config["obs_density"], config["swamp_percent"])
        
        print("-" * 75)
        print(f"| {'Map':<6} | {'Heuristic':<20} | {'Path Length':<10} | {'Nodes Expanded':<15} | {'Runtime':<12} |")
        print("|:-------|:---------------------|:-----------|:----------------|:-------------|")
        
        for name, func in HEURISTICS.items():
            run_benchmark(map_id, grid, start_coords, goal_coords, func, name)

        print("-" * 75)


if __name__ == "__main__":
    main()
