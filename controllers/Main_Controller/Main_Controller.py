from controller import Robot
import heapq
import math
import sys
import time

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Read ROBOT_ID from controller arguments
ROBOT_ID = None
for arg in sys.argv:
    if arg.startswith("ROBOT_ID="):
        ROBOT_ID = int(arg.split("=")[1])

if ROBOT_ID is None:
    raise ValueError("Please set ROBOT_ID as a controller argument.")

# Motors - done
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Define movement directions (x, y) - done
MOVES = [(0, 0.25), (0.25, 0), (0, -0.25), (-0.25, 0)]
DIRECTIONS = ['N', 'E', 'S', 'W']
DIRECTION_TO_VECTOR = {'N': (0.0, 0.25), 'E': (0.25, 0.0), 'S': (0.0, -0.25), 'W': (-0.25, 0.0)}

# Set start and goal based on ROBOT_ID
if ROBOT_ID == 1:
    start = (-0.375, -0.375)
    goal = (0.375, 0.375)
elif ROBOT_ID == 2:
    start = (-0.375, -0.125)
    goal = (0.375, 0.125)

# Obstacles (static) - done
obstacles = {
    (-0.125, 0.375),
    (0.125, -0.125),
    (-0.375, 0.125)
    
}

# Reservation table (shared via file for simplicity)
RES_FILE = "/tmp/reservations.txt"
MAX_STEPS = 50  # Max time steps to plan

# Initialize reservation file if first robot
if ROBOT_ID == 1:
    with open(RES_FILE, "w") as f:
        f.write("")

# Heuristic for A* - done
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# A* with reservation table
def a_star_with_reservations(start, goal):
    open_set = []
    heapq.heappush(open_set, (0, 0, start))
    came_from = {}
    g_score = {(start, 0): 0}
    f_score = {(start, 0): heuristic(start, goal)}

    reservations = load_reservations()

    while open_set:
        _, t, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while (current, t) in came_from:
                path.append((current, t))
                current, t = came_from[(current, t)]
            path.append((start, 0))
            path.reverse()
            return [pos for pos, _ in path]

        for dx, dy in MOVES:
            neighbor = (round(current[0] + dx, 3), round(current[1] + dy, 3))
            next_t = t + 1
            if -0.5 <= neighbor[0] < 0.5 and -0.5 <= neighbor[1] < 0.5 and neighbor not in obstacles:
                if (neighbor, next_t) in reservations:
                    continue
                temp_g = g_score.get((current, t), float("inf")) + 1
                if (neighbor, next_t) not in g_score or temp_g < g_score[(neighbor, next_t)]:
                    came_from[(neighbor, next_t)] = (current, t)
                    g_score[(neighbor, next_t)] = temp_g
                    f_score[(neighbor, next_t)] = temp_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[(neighbor, next_t)], next_t, neighbor))
    return None

# Load reservations from file
def load_reservations():
    res = set()
    try:
        with open(RES_FILE, "r") as f:
            for line in f:
                parts = line.strip().split(",")
                if len(parts) == 3:
                    x, y, t = float(parts[0]), float(parts[1]), int(parts[2])
                    res.add(((x, y), t))
    except FileNotFoundError:
        pass
    return res

def reserve_path(path):
    with open(RES_FILE, "a") as f:
        for t, pos in enumerate(path):
            f.write(f"{pos[0]},{pos[1]},{t}\n")

def get_direction(from_pos, to_pos):
    dx = round(to_pos[0] - from_pos[0], 3)
    dy = round(to_pos[1] - from_pos[1], 3)
    for dir, vec in DIRECTION_TO_VECTOR.items():
        if (dx, dy) == vec:
            return dir
    return None

# Rotation logic
current_direction = 'E'
def rotate_to_direction(target_direction):
    global current_direction
    if current_direction == target_direction:
        return

    current_idx = DIRECTIONS.index(current_direction)
    target_idx = DIRECTIONS.index(target_direction)
    diff = (target_idx - current_idx) % 4

    if diff == 1: 
        left_motor.setVelocity(3.0)
        right_motor.setVelocity(-3.0)
    elif diff == 3:  
        left_motor.setVelocity(-3.0)
        right_motor.setVelocity(3.0)
    elif diff == 2:  
        left_motor.setVelocity(3.0)
        right_motor.setVelocity(-3.0)

    steps = int((0.75 if diff in [1, 3] else 1.5) * 1000 / timestep)
    for _ in range(steps):
        robot.step(timestep)

    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    current_direction = target_direction

# Move forward
def move_forward():
    left_motor.setVelocity(3.0)
    right_motor.setVelocity(3.0)
    steps = int(4.2 * 1000 / timestep)
    for _ in range(steps):
        robot.step(timestep)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

# Plan and move
path = None
while robot.step(timestep) != -1:
    if path is None:
        path = a_star_with_reservations(start, goal)
        if path:
            print(f"Robot {ROBOT_ID} path: {path}")
            reserve_path(path)
            step_index = 1
        else:
            print(f"Robot {ROBOT_ID} could not find a path.")
            break
    elif step_index < len(path):
        prev = path[step_index - 1]
        curr = path[step_index]
        dir = get_direction(prev, curr)
        if dir:
            rotate_to_direction(dir)
            move_forward()
        step_index += 1
    else:
        break
