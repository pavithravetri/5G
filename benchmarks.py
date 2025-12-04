# benchmarks.py
import time
from grid import Grid
from astar import astar

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

if __name__ == "__main__":
    run_benchmarks()
