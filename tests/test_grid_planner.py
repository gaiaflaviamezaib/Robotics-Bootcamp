"""Tests for the grid route planner in the two smart_manhattan navigation scripts.

The navigation scripts drive the robot as soon as they run, so these tests load
only the grid constants and the pure planning functions from each file. No
hardware is needed.

Run from the repository root with: python -m pytest
"""
import ast
from pathlib import Path

import pytest

NAV_DIR = Path(__file__).resolve().parents[1] / "navigation"
CONSTANTS = {"GRID_WIDTH", "VALID_NODES", "EDGES"}
FUNCTIONS = {"get_neighbors", "normalize_dir", "bfs_path", "path_to_instructions"}
STRAIGHT, TURN, U_TURN = 1.0, 1.5, 3.0


def load_planner(filename):
    tree = ast.parse((NAV_DIR / filename).read_text(encoding="utf-8"))
    keep = [
        node for node in tree.body
        if (isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS)
        or (isinstance(node, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id in CONSTANTS for t in node.targets))
    ]
    namespace = {}
    exec("import heapq\nimport itertools", namespace)
    exec(compile(ast.Module(body=keep, type_ignores=[]), filename, "exec"), namespace)
    return namespace


@pytest.fixture(params=["smart_manhattan.py", "smart_manhattan_obstacles.py"])
def planner(request):
    return load_planner(request.param)


def route_cost(path, start_dir, normalize_dir):
    cost, direction = 0.0, start_dir
    for a, b in zip(path, path[1:]):
        move = normalize_dir(b[0] - a[0], b[1] - a[1])
        dot = direction[0] * move[0] + direction[1] * move[1]
        cost += STRAIGHT if dot > 0 else U_TURN if dot < 0 else TURN
        direction = move
    return cost


def simple_paths(start, end, get_neighbors, path=None):
    path = path or [start]
    if path[-1] == end:
        yield path
        return
    for nxt in get_neighbors(path[-1]):
        if nxt not in path:
            yield from simple_paths(start, end, get_neighbors, path + [nxt])


def test_grid_edges_are_valid(planner):
    for edge in planner["EDGES"]:
        a, b = tuple(edge)
        assert a in planner["VALID_NODES"] and b in planner["VALID_NODES"]
        assert a[0] == b[0] or a[1] == b[1], "edges follow the grid lines"


def test_start_equals_end(planner):
    assert planner["bfs_path"]((1, 0.0), (1, 0.0)) == [(1, 0.0)]


def test_straight_route_along_bottom_row(planner):
    path = planner["bfs_path"]((0, 0.0), (3, 0.0), (1, 0))
    assert path == [(0, 0.0), (1, 0.0), (2, 0.0), (3, 0.0)]


@pytest.mark.parametrize("start_dir", [(1, 0), (0, 1), (-1, 0), (0, -1)])
def test_routes_are_connected_and_cheapest(planner, start_dir):
    nodes = sorted(planner["VALID_NODES"])
    neighbors = planner["get_neighbors"]
    for start in nodes:
        for end in nodes:
            if start == end:
                continue
            path = planner["bfs_path"](start, end, start_dir)
            assert path[0] == start and path[-1] == end
            for a, b in zip(path, path[1:]):
                assert frozenset({a, b}) in planner["EDGES"]
            best = min(
                route_cost(p, start_dir, planner["normalize_dir"])
                for p in simple_paths(start, end, neighbors)
            )
            assert route_cost(path, start_dir, planner["normalize_dir"]) <= best


def test_blocked_nodes_are_avoided():
    planner = load_planner("smart_manhattan_obstacles.py")
    blocked = {(1, 0.0)}
    path = planner["bfs_path"]((0, 0.0), (3, 0.0), (1, 0), blocked)
    assert path[0] == (0, 0.0) and path[-1] == (3, 0.0)
    assert not blocked & set(path)

    # Blocking both ways out of (0, 0) leaves no route.
    assert planner["bfs_path"]((0, 0.0), (3, 0.0), (1, 0), {(1, 0.0), (0, 1.0)}) == []


def test_instructions_for_short_paths(planner):
    to_instructions = planner["path_to_instructions"]
    assert to_instructions([(0, 0.0)], (1, 0)) == ["stop"]
    # A U-turn becomes two left turns.
    assert to_instructions([(1, 0.0), (0, 0.0)], (1, 0)) == ["left", "left", "stop"]


def test_instructions_turn_right_at_corner(planner):
    # Start at (0, 0) facing north, go up, then turn east at (0, 1).
    path = [(0, 0.0), (0, 1.0), (1, 1.0)]
    assert planner["path_to_instructions"](path, (0, 1)) == ["right", "stop"]


def test_instructions_skip_t_junctions_when_going_straight(planner):
    # The bottom row only has branches on one side, which the line sensors
    # do not report as an intersection, so no 'straight' is emitted.
    path = [(0, 0.0), (1, 0.0), (2, 0.0), (3, 0.0)]
    assert planner["path_to_instructions"](path, (1, 0)) == ["stop"]
