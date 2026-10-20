"""Unit tests matching TC-U01..TC-U03 in the report.   Run:  python test_astar.py"""
import os, sys
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import drone_rescue_simulation as D

def empty_grid():
    return [[0] * D.GRID_WIDTH for _ in range(D.GRID_HEIGHT)]

def test_u01_optimal_on_empty_grid():
    path = D.astar(0, 0, 5, 5, empty_grid())
    assert path is not None and len(path) - 1 == 10          # 10 steps = Manhattan distance

def test_u02_detours_around_wall():
    g = empty_grid()
    for x in range(0, 5): g[3][x] = 1                         # wall at y=3, open from x>=5
    path = D.astar(2, 2, 2, 4, g)
    assert path is not None
    assert all(g[y][x] != 1 for x, y in path)                 # never enters a hazard
    assert len(path) - 1 == 8                                 # 3 right + 2 down + 3 left

def test_u03_unreachable_goal_returns_none():
    g = empty_grid()
    for x, y in [(5, 4), (5, 6), (4, 5), (6, 5)]: g[y][x] = 1  # goal (5,5) boxed in
    assert D.astar(0, 0, 5, 5, g) is None

def test_extra_start_equals_goal():
    assert D.astar(3, 3, 3, 3, empty_grid()) == [(3, 3)]

if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t(); print("PASS", t.__name__)
    print(f"{len(tests)}/{len(tests)} passed")
