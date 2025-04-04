from controller import Robot
import heapq
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

MOVES = [(0, 0.25), (0.25, 0), (0, -0.25), (-0.25, 0)]
# GRID_SIZE = 4

start = (-0.375, -0.375) 
goal = (0.375, 0.375) 

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for dx, dy in MOVES:
            neighbor = (current[0] + dx, current[1] + dy)
            if -0.5 <= neighbor[0] < 0.5 and -0.5 <= neighbor[1] < 0.5:
                temp_g = g_score[current] + 1
                if neighbor not in g_score or temp_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = temp_g
                    f_score[neighbor] = temp_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None

DIRECTION_TO_VECTOR = {'N': (0.0, 0.25), 'E': (-0.25, 0.0), 'S': (0.0, -0.25), 'W': (0.25, 0.0)}

def get_direction(from_pos, to_pos):
    dx = (to_pos[0] - from_pos[0])
    dy = (to_pos[1] - from_pos[1])
    for dir, vec in DIRECTION_TO_VECTOR.items():
        if (dx, dy) == vec:
            return dir
    
    print(f"Unmatched direction: from {from_pos} to {to_pos} → ({dx}, {dy})")

    return None

current_direction = 'E'
DIRECTIONS = ['N', 'E', 'S', 'W']

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

def move_to_target(prev_grid, next_grid):
    # target_position = grid_to_position(*next_grid)

    target_dir = get_direction(prev_grid, next_grid)
    rotate_to_direction(target_dir)

    # Move forward
    left_motor.setVelocity(3.0)
    right_motor.setVelocity(3.0)

    # Dummy movement delay (tune if using real sensors)
    steps = int(1.2 * 1000 / timestep)
    for _ in range(steps):
        robot.step(timestep)

    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

path = a_star(start, goal)
if path:
    print("Path:", path)
    for i in range(1, len(path)):
        move_to_target(path[i - 1], path[i])
else:
    print("No path found")
