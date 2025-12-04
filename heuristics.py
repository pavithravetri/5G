# heuristics.py
import math

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def euclidean(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

# Custom heuristic: weighted Manhattan + terrain penalty
def custom(a, b, terrain_cost):
    base = manhattan(a, b)
    penalty = 0.1 * terrain_cost
    return base + penalty
