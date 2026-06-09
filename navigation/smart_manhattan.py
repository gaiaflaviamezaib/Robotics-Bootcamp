import time
import sys
import os
import heapq
import itertools
from collections import deque

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from bootcamp_tasks.robotControlClass import robotControl

# =========================================================================
# PART 1 — BRAIN (Grid model, pathfinding, user interface)
# =========================================================================

GRID_WIDTH = 4

# Grid layout (irregular — coordinates match the physical track):
#
#   (0,1) ========= (1,1) ========= (2,1) --- (3,1)
#     ‖                ‖               |         |
#     ‖                ‖           (2,0.5) -- (3,0.5)
#     ‖                ‖               |         |
#   (0,0) ========= (1,0) ========= (2,0) --- (3,0)
#
# ====  : double-width segment (longer physical road)
# ---   : single-width segment

VALID_NODES = {
    (0, 0.0), (1, 0.0), (2, 0.0), (3, 0.0),
                        (2, 0.5), (3, 0.5),
    (0, 1.0), (1, 1.0), (2, 1.0), (3, 1.0),
}

EDGES = {
    frozenset({(0, 0.0), (1, 0.0)}),
    frozenset({(1, 0.0), (2, 0.0)}),
    frozenset({(2, 0.0), (3, 0.0)}),
    frozenset({(2, 0.5), (3, 0.5)}),
    frozenset({(0, 1.0), (1, 1.0)}),
    frozenset({(1, 1.0), (2, 1.0)}),
    frozenset({(2, 1.0), (3, 1.0)}),
    frozenset({(0, 0.0), (0, 1.0)}),
    frozenset({(1, 0.0), (1, 1.0)}),
    frozenset({(2, 0.0), (2, 0.5)}),
    frozenset({(2, 0.5), (2, 1.0)}),
    frozenset({(3, 0.0), (3, 0.5)}),
    frozenset({(3, 0.5), (3, 1.0)}),
}


def get_neighbors(node):
    """Return all nodes directly connected to the given node."""
    neighbors = []
    for edge in EDGES:
        if node in edge:
            neighbors.append(list(edge - {node})[0])
    return neighbors


def normalize_dir(dx, dy):
    """Normalize a direction vector to unit steps (-1, 0, or 1)."""
    return (
        1 if dx > 0 else (-1 if dx < 0 else 0),
        1 if dy > 0 else (-1 if dy < 0 else 0),
    )


def display_grid(start_x, start_y, end_x, end_y, path=None):
    """Print the grid in the terminal with start, end, and path highlighted."""
    print(f"\n  ============== GRID ==============\n")

    def node_str(x, y):
        if (x, y) not in VALID_NODES:
            return "   "
        if x == start_x and y == start_y:
            return "[S]"
        elif x == end_x and y == end_y:
            return "[E]"
        elif path and (x, y) in path:
            return " x "
        return " O "

    def h_edge(x1, y, x2):
        return "---" if frozenset({(float(x1), float(y)), (float(x2), float(y))}) in EDGES else "   "

    def v_edge(x, y1, y2):
        if frozenset({(float(x), float(y1)), (float(x), float(y2))}) in EDGES:
            return "|"
        if frozenset({(float(x), 0.0), (float(x), 1.0)}) in EDGES:
            return "|"
        return " "

    # Top row (y=1.0)
    line = "  "
    for x in range(GRID_WIDTH):
        line += node_str(x, 1.0)
        if x < GRID_WIDTH - 1:
            line += h_edge(x, 1.0, x + 1)
    print(line)

    # Vertical separators between y=0.5 and y=1.0
    sep = "   "
    for x in range(GRID_WIDTH):
        sep += v_edge(x, 0.5, 1.0) + "     "
    print(sep)

    # Middle row (y=0.5)
    line = "  "
    for x in range(GRID_WIDTH):
        line += node_str(x, 0.5)
        if x < GRID_WIDTH - 1:
            line += h_edge(x, 0.5, x + 1)
    print(line)

    # Vertical separators between y=0.0 and y=0.5
    sep = "   "
    for x in range(GRID_WIDTH):
        sep += v_edge(x, 0.0, 0.5) + "     "
    print(sep)

    # Bottom row (y=0.0)
    line = "  "
    for x in range(GRID_WIDTH):
        line += node_str(x, 0.0)
        if x < GRID_WIDTH - 1:
            line += h_edge(x, 0.0, x + 1)
    print(line)

    print("\n  Legend: [S]=Start  [E]=End  x=Path\n")


def parse_coord(val):
    """Parse user input into (x, y) floats. Accepts '10', '1,0', '2,mid'."""
    val = val.lower().replace("mid", "0.5")
    if len(val) == 2 and ',' not in val:
        return float(val[0]), float(val[1])
    parts = val.split(',')
    return float(parts[0]), float(parts[1])


def get_coordinates(prompt):
    """Prompt the user until they enter a valid grid node."""
    while True:
        val = input(prompt).strip()
        try:
            x, y = parse_coord(val)
            for nx, ny in VALID_NODES:
                if abs(x - nx) < 0.1 and abs(y - ny) < 0.1:
                    return nx, ny
            print(f"Error: ({val}) is not a valid node.")
            print("Valid nodes: (0,1) (1,1) (2,1) (3,1) (2,mid) (3,mid) (0,0) (1,0) (2,0) (3,0)")
        except Exception:
            print("Invalid format. Use 'x,y' — e.g. 1,1 or 2,mid")


def bfs_path(start, end, start_dir=None):
    """
    Find the lowest-cost path from start to end using Dijkstra's algorithm.
    Turn cost is 1.5x, straight is 1x, U-turn is 3x.
    """
    if start == end:
        return [start]

    if not start_dir:
        start_dir = (1, 0)  # Default facing East

    counter = itertools.count()
    pq = [(0, next(counter), start, start_dir, [start])]
    visited = {}  # (node, direction) -> minimum cost seen

    while pq:
        cost, _, curr, curr_dir, path = heapq.heappop(pq)

        if curr == end:
            return path

        state = (curr, curr_dir)
        if state in visited and visited[state] <= cost:
            continue
        visited[state] = cost

        for neighbor in get_neighbors(curr):
            dx = neighbor[0] - curr[0]
            dy = neighbor[1] - curr[1]
            move_dir = normalize_dir(dx, dy)

            cx, cy = curr_dir
            mx, my = move_dir
            dot = cx * mx + cy * my

            if dot > 0:
                new_cost = cost + 1.0   # Straight ahead
            elif dot < 0:
                new_cost = cost + 3.0   # U-turn (penalized)
            else:
                new_cost = cost + 1.5   # Left or right turn

            heapq.heappush(pq, (new_cost, next(counter), neighbor, move_dir, path + [neighbor]))

    return []


def path_to_instructions(path, start_dir):
    """
    Convert a node path into a list of driving instructions
    ('straight', 'left', 'right', 'stop').

    Only emits instructions for nodes the robot will actually detect as
    [1,1,1] intersections (branches on BOTH perpendicular sides).
    T-intersections (branch on one side only) are invisible to the robot
    and are skipped.

    The first instruction is dropped because the robot starts already aligned
    on the first segment and drives blind to the first intersection.
    """
    if len(path) < 2:
        return ["stop"]

    def is_detectable(node, travel_dir):
        """Check if a node has neighbors on BOTH perpendicular sides."""
        tx, ty = travel_dir
        # Perpendicular directions: rotate 90° CW and CCW
        perp_left  = (-ty, tx)
        perp_right = (ty, -tx)

        left_neighbor  = (node[0] + perp_left[0],  node[1] + perp_left[1])
        right_neighbor = (node[0] + perp_right[0], node[1] + perp_right[1])

        # Check if both perpendicular neighbors exist as valid nodes with edges
        has_left  = frozenset({node, left_neighbor}) in EDGES
        has_right = frozenset({node, right_neighbor}) in EDGES
        return has_left and has_right

    instructions = []
    current_dir = normalize_dir(*start_dir)
    first_emitted = False  # Track which is the first instruction (to drop it)

    for i in range(len(path) - 1):
        curr_x, curr_y = path[i]
        next_x, next_y = path[i + 1]
        move_dir = normalize_dir(next_x - curr_x, next_y - curr_y)

        cx, cy = current_dir
        mx, my = move_dir
        cross = cx * my - cy * mx
        dot = cx * mx + cy * my

        # Determine the instruction for this transition
        if dot > 0:
            action = "straight"
        elif cross > 0:
            action = "left"
        elif cross < 0:
            action = "right"
        else:
            # U-turn — always detectable (robot arrives at a dead end)
            instructions.append("left")
            instructions.append("left")
            current_dir = move_dir
            first_emitted = True
            continue

        # Skip the very first segment (robot starts pre-aligned)
        if not first_emitted:
            first_emitted = True
            current_dir = move_dir
            continue

        # For 'straight': only emit if this node is a detectable intersection
        # Turns are always at detectable nodes (the robot must decide)
        if action == "straight":
            node = path[i]
            if not is_detectable(node, current_dir):
                # Robot won't see this as [1,1,1] — skip it
                current_dir = move_dir
                continue

        instructions.append(action)
        current_dir = move_dir

    instructions.append("stop")
    return instructions



# =========================================================================
# STARTUP — Ask user for start/end, compute path, wait for go
# =========================================================================

print("\nWelcome to the Smart Manhattan Navigator!")
start_x, start_y = get_coordinates("Enter start coordinates (e.g. 0,0): ")
end_x, end_y     = get_coordinates("Enter destination   (e.g. 2,1): ")

display_grid(start_x, start_y, end_x, end_y)

path = bfs_path((start_x, start_y), (end_x, end_y))
if not path:
    print("Error: no path found!")
    sys.exit(1)

display_grid(start_x, start_y, end_x, end_y, path)

if len(path) > 1:
    start_dir_vector = normalize_dir(path[1][0] - path[0][0], path[1][1] - path[0][1])
else:
    start_dir_vector = (1, 0)

PATH_INSTRUCTIONS = path_to_instructions(path, start_dir_vector)
print(f"Path computed : {path}")
print(f"Instructions  : {PATH_INSTRUCTIONS}")
print(f"\n>>> Place the robot at ({start_x},{start_y}) facing "
      f"({path[1][0] if len(path) > 1 else start_x},"
      f"{path[1][1] if len(path) > 1 else start_y}) <<<")
input(">>> Press ENTER to start the mission <<<")


# =========================================================================
# PART 2 — BODY (turning maneuvers + PID line following)
# =========================================================================

robot = robotControl()


def turn_right(robot, speed=30):
    """
    Execute a right-angle right turn using a 4-step Ackermann maneuver.

    Step 0 — Re-center on intersection (sensor-guided reverse).
    Step 1 — Wheels full-right, drive forward to swing the rear left.
    Step 2 — Wheels full-left,  drive backward to swing the nose right.
    Step 3 — Wheels right, short forward burst to aim at the new lane.
    Step 4 — Sensor-guided forward creep until the line is detected.
    """
    print("\n>>> Turning right...")

    # --- Step 0: fine-center on intersection ---
    robot.turnWheels(0)
    robot.runMotors(1, 28)      # Short impulse to break static friction
    time.sleep(0.05)
    robot.runMotors(1, 23)      # Very slow reverse to find the intersection

    # Wait until sensors leave the intersection cross
    # (stop early if we overshoot back to the previous intersection)
    timeout = time.time() + 0.3
    while time.time() < timeout:
        state = robot.getInfraredState(False)
        if state == [0, 0, 0]:
            break
        if state == [1, 1, 1]:          # Hit previous intersection — stop
            robot.stopMotors()
            time.sleep(0.5)
            break
        time.sleep(0.01)

    # Wait until a side sensor catches the cross again (robot is centered)
    timeout = time.time() + 0.3
    while time.time() < timeout:
        state = robot.getInfraredState(False)
        if state[0] == 1 or state[2] == 1:
            break
        if state == [1, 1, 1]:          # Hit previous intersection — stop
            robot.stopMotors()
            time.sleep(0.5)
            break
        time.sleep(0.01)

    # Extra dwell so the wheel axle (not just sensors) sits on the cross
    # (abort if we reach the previous intersection)
    dwell_end = time.time() + 0.5
    while time.time() < dwell_end:
        if robot.getInfraredState(False) == [1, 1, 1]:
            break
        time.sleep(0.01)
    robot.stopMotors()
    time.sleep(0.5)

    # --- Step 1: wheels right, drive forward ---
    robot.turnWheels(-120)
    time.sleep(0.3)
    robot.runMotors(0, speed)
    time.sleep(1.2)
    robot.stopMotors()
    time.sleep(0.3)

    # --- Step 2: wheels left, drive backward ---
    robot.turnWheels(120)
    time.sleep(0.3)
    robot.runMotors(1, speed)
    time.sleep(0.93)
    robot.stopMotors()
    time.sleep(0.3)

    # --- Step 3: wheels right, short forward burst ---
    robot.turnWheels(-80)
    time.sleep(0.3)
    robot.runMotors(0, speed)
    time.sleep(0.25)

    # --- Step 4: sensor-guided alignment ---
    # Timer starts on first line detection and does NOT reset.
    # 0.3 s of any stable detection is enough to confirm the line.
    robot.turnWheels(-45)
    time.sleep(0.2)
    line_found_time  = None
    last_align_steer = -45
    maneuver_timeout = time.time() + 3.0    # Safety cap: abort after 3 s

    while time.time() < maneuver_timeout:
        state = robot.getInfraredState(False)

        if state == [0, 1, 0] or state == [1, 1, 1]:
            last_align_steer = 0
            if line_found_time is None:
                line_found_time = time.time()
        elif state[0] == 1:
            last_align_steer = 60
            if line_found_time is None:
                line_found_time = time.time()
        elif state[2] == 1:
            last_align_steer = -60
            if line_found_time is None:
                line_found_time = time.time()
        # [0,0,0]: line not yet found — keep sweeping, do not start timer

        robot.turnWheels(last_align_steer)

        # Stop once the line has been detected for 0.3 s
        if line_found_time is not None and time.time() - line_found_time > 0.3:
            break

        robot.runMotors(0, 23)
        time.sleep(0.05)

    robot.stopMotors()
    robot.turnWheels(0)
    time.sleep(0.2)
    print(">>> Right turn complete.\n")


def turn_left(robot, speed=30):
    """
    Execute a right-angle left turn using a 4-step Ackermann maneuver.

    Step 0 — Re-center on intersection (sensor-guided reverse).
    Step 1 — Wheels full-left,  drive forward to swing the rear right.
    Step 2 — Wheels full-right, drive backward to swing the nose left.
    Step 3 — Wheels left, short forward burst to aim at the new lane.
    Step 4 — Sensor-guided forward creep until the line is detected.
    """
    print("\n>>> Turning left...")

    # --- Step 0: fine-center on intersection ---
    robot.turnWheels(0)
    robot.runMotors(1, 28)      # Short impulse to break static friction
    time.sleep(0.05)
    robot.runMotors(1, 23)      # Very slow reverse to find the intersection

    timeout = time.time() + 0.3
    while time.time() < timeout:
        state = robot.getInfraredState(False)
        if state == [0, 0, 0]:
            break
        if state == [1, 1, 1]:          # Hit previous intersection — stop
            robot.stopMotors()
            time.sleep(0.5)
            break
        time.sleep(0.01)

    timeout = time.time() + 0.3
    while time.time() < timeout:
        state = robot.getInfraredState(False)
        if state[0] == 1 or state[2] == 1:
            break
        if state == [1, 1, 1]:          # Hit previous intersection — stop
            robot.stopMotors()
            time.sleep(0.5)
            break
        time.sleep(0.01)

    dwell_end = time.time() + 0.5
    while time.time() < dwell_end:
        if robot.getInfraredState(False) == [1, 1, 1]:
            break
        time.sleep(0.01)
    robot.stopMotors()
    time.sleep(0.5)

    # --- Step 1: wheels left, drive forward ---
    robot.turnWheels(120)
    time.sleep(0.3)
    robot.runMotors(0, speed)
    time.sleep(0.9)
    robot.stopMotors()
    time.sleep(0.3)

    # --- Step 2: wheels right, drive backward ---
    robot.turnWheels(-110)
    time.sleep(0.3)
    robot.runMotors(1, speed)
    time.sleep(1)
    robot.stopMotors()
    time.sleep(0.3)

    # --- Step 3: wheels left, short forward burst ---
    robot.turnWheels(80)
    time.sleep(0.3)
    robot.runMotors(0, speed)
    time.sleep(0.25)

    # --- Step 4: sensor-guided alignment ---
    # Timer starts on first line detection and does NOT reset.
    # 0.3 s of any stable detection is enough to confirm the line.
    robot.turnWheels(45)
    time.sleep(0.2)
    line_found_time  = None
    last_align_steer = 45
    maneuver_timeout = time.time() + 3.0    # Safety cap: abort after 3 s

    while time.time() < maneuver_timeout:
        state = robot.getInfraredState(False)

        if state == [0, 1, 0] or state == [1, 1, 1]:
            last_align_steer = 0
            if line_found_time is None:
                line_found_time = time.time()
        elif state[0] == 1:
            last_align_steer = 60
            if line_found_time is None:
                line_found_time = time.time()
        elif state[2] == 1:
            last_align_steer = -60
            if line_found_time is None:
                line_found_time = time.time()
        # [0,0,0]: line not yet found — keep sweeping, do not start timer

        robot.turnWheels(last_align_steer)

        # Stop once the line has been detected for 0.3 s
        if line_found_time is not None and time.time() - line_found_time > 0.3:
            break

        robot.runMotors(0, 23)
        time.sleep(0.05)

    robot.stopMotors()
    robot.turnWheels(0)
    time.sleep(0.2)
    print(">>> Left turn complete.\n")


# =========================================================================
# PID LINE-FOLLOWING CONFIGURATION
# =========================================================================

Kp = 7.5    # Proportional gain
Ki = 2.5    # Integral gain
Kd = 2.5    # Derivative gain

turn_multiplier = 1
pastStates = []
lastError  = 0


def getError(infraState, lastError):
    """
    Map an infrared sensor state to a signed error value.
    Positive = robot drifted right; negative = robot drifted left.
    """
    if   infraState == [1, 0, 1]: return  0.0
    elif infraState == [1, 0, 0]: return  1.0
    elif infraState == [1, 1, 0]: return  2.0
    elif infraState == [0, 1, 0]:
        # On-center: continue in the same direction as before
        if   lastError > 0: return  0.5
        elif lastError < 0: return -0.5
        else:               return  0.0
    elif infraState == [0, 1, 1]: return -2.0
    elif infraState == [0, 0, 1]: return -1.0
    elif infraState == [1, 1, 1]: return lastError   # Intersection — hold last error
    elif infraState == [0, 0, 0]: return lastError   # Line lost — hold last error
    else:                         return lastError


# =========================================================================
# MAIN LOOP
# =========================================================================

print("=" * 40)
print("Motors starting. Navigation active!")
print("=" * 40)

print("Centering wheels...")
robot.turnWheels(0)
time.sleep(0.5)

print("Start impulse to overcome static friction...")
robot.runMotors(0, 27)
time.sleep(0.2)

programStartTime     = time.time()
lastIntersectionTime = time.time() - 10
instructions         = list(PATH_INSTRUCTIONS)

try:
    while True:
        now = time.perf_counter()
        currentInfraState = robot.getInfraredState(False)

        # Ignore intersections for the first 0.5 s (startup immunity)
        is_startup_immunity = time.time() - programStartTime < 0.5

        # ---- Intersection detected ----
        # Peek at the next action to know which sensor pattern to expect:
        #   [1,1,1] = full 4-way or T coming from below (classic)
        #   [1,1,0] = T-intersection with branch on the LEFT only  → need left turn
        #   [0,1,1] = T-intersection with branch on the RIGHT only → need right turn
        next_action_peek = instructions[0] if instructions else None

        is_full_cross  = currentInfraState == [1, 1, 1]
        is_t_left      = (currentInfraState == [1, 1, 0]
                          and next_action_peek in ("left", "stop"))
        is_t_right     = (currentInfraState == [0, 1, 1]
                          and next_action_peek in ("right", "stop"))

        if ((is_full_cross or is_t_left or is_t_right)
                and not is_startup_immunity
                and time.time() - lastIntersectionTime > 0.5):

            robot.stopMotors()
            robot.turnWheels(0)
            time.sleep(0.5)

            if not instructions:
                print("No more instructions. Stopping.")
                break

            nextAction = instructions.pop(0)
            print(f"\n[INTERSECTION] -> Action: {nextAction}")

            if nextAction == "stop":
                print("Destination reached!")
                break

            elif nextAction == "straight":
                print("Going straight...")
                robot.runMotors(0, 23)
                robot.turnWheels(0)
                time.sleep(0.4)

            elif nextAction == "left":
                turn_left(robot)

            elif nextAction == "right":
                turn_right(robot)

            lastIntersectionTime = time.time()
            continue


        # ---- PID line following ----
        error = getError(currentInfraState, lastError)

        proportional = error

        pastStates.append(error)
        if len(pastStates) > 10:
            pastStates.pop(0)

        integral = sum(pastStates)
        integral = max(-40, min(40, integral))

        derivative = error - lastError
        lastError  = error

        output = Kp * proportional + Ki * integral + Kd * derivative
        output  = max(-25, min(25, output))

        robot.turnWheels(output * turn_multiplier)
        robot.runMotors(0, 20)

        # Maintain ~100 Hz loop rate
        dt = time.perf_counter() - now
        if dt < 0.01:
            time.sleep(0.01 - dt)

    print("\n=== MISSION COMPLETE ===")

except KeyboardInterrupt:
    print("\nStopped by user (Ctrl+C).")
finally:
    robot.turnWheels(0)
    robot.resetAll()
    print("Robot stopped safely.")
