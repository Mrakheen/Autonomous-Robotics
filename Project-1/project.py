#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Constants for the workspace and grid dimensions in meters
WORKSPACE_WIDTH = 4.88
WORKSPACE_HEIGHT = 3.05  
TILE_SIZE = 0.305*1.5 
GRID_WIDTH = int(WORKSPACE_WIDTH / TILE_SIZE)
GRID_HEIGHT = int(WORKSPACE_HEIGHT / TILE_SIZE)

# Error angle constants after continuous experiments with gyro
ERROR_L = 9
ERROR_R = -9

angle = 0


# Define grid cell
class GridCell:
    def __init__(self, x, y):
        self.row = x
        self.col = y
        self.x = 0
        self.y = 0
        self.direction = 'X'
        self.distance = 0
        self.visited = False
        self.is_start = False
        self.is_end = False
        self.is_obstacle = False
    
    def set_x(self, x):
        self.x = x
        
    def set_y(self, y):
        self.y = y

    def set_visited(self):
        self.visited = True
        
    def set_start(self):
        self.is_start = True
        self.visited = True

    def set_goal(self):
        self.is_end = True

    def set_obstacle(self):
        self.distance = -1
        self.is_obstacle = True

    def set_distance(self, value):
        self.distance = value

# Define the grid
class Grid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.start = None
        self.goal = None
        self.grid = [[GridCell(row, col) for col in range(width)] for row in range(height)]

    # Configures the grid as a XY coordinate plane       
    def set_grid_xy(self):
        for i in range(self.height):
            for j in range(self.width):
                self.grid[i][j].set_x(j)
                self.grid[i][j].set_y(self.height - 1 - i)

    def set_start(self, start):
        calculated_X = int(start[0] / TILE_SIZE)
        calculated_Y = int(start[1] / TILE_SIZE)
        for i in range(self.height):
            for j in range(self.width):
                Cell = self.grid[i][j]
                if Cell.x == calculated_X and Cell.y == calculated_Y:
                    Cell.set_start()
                    self.start = Cell
        
    def set_goal(self, goal):
        calculated_X = int(goal[0] / TILE_SIZE)
        calculated_Y = int(goal[1] / TILE_SIZE)
        for i in range(self.height):
            for j in range(self.width):
                Cell = self.grid[i][j]
                if Cell.x == calculated_X and Cell.y == calculated_Y:
                    Cell.set_goal()
                    self.goal = Cell
    
    def set_obstacles(self, obstacles):
        for obstacle in obstacles:
            if obstacle[0] == -1 or obstacle[1] == -1:
                continue
            calculated_X = int(obstacle[0] / TILE_SIZE)
            calculated_Y = int(obstacle[1] / TILE_SIZE)
            for i in range(self.height):
                for j in range(self.width):
                    Cell = self.grid[i][j]
                    if Cell.x == calculated_X and Cell.y == calculated_Y:
                        Cell.set_obstacle()

    # Calculates the Manhattan Distance for each cell on the grid using a queue
    def set_manhattan_values(self):
        visited = set()
        queue = [(self.goal, 0)]

        while queue:
            cell, distance = queue.pop(0)
            cell.set_distance(distance)
            visited.add(cell)

            for neighbor in self.find_neighboring_cells(cell):
                if neighbor not in visited and not neighbor.is_obstacle:
                    queue.append((neighbor, distance + 1))
                    visited.add(neighbor)

    # Finds all adjecent cells of a given cell and sets its orientation
    def find_neighboring_cells(self, Cell):
        neighbors = []
        if 0 <= Cell.x - 1 < self.width:
            self.grid[Cell.row][Cell.col - 1].direction = 'W'
            neighbors.append(self.grid[Cell.row][Cell.col - 1])
        if 0 <= Cell.x + 1 < self.width:
            self.grid[Cell.row][Cell.col + 1].direction = 'E'
            neighbors.append(self.grid[Cell.row][Cell.col + 1])
        if 0 <= Cell.y - 1 < self.height:
            self.grid[Cell.row + 1][Cell.col].direction = 'S'
            neighbors.append(self.grid[Cell.row + 1][Cell.col])
        if 0 <= Cell.y + 1 < self.height:
            self.grid[Cell.row - 1][Cell.col].direction = 'N'
            neighbors.append(self.grid[Cell.row - 1][Cell.col])
        return neighbors
    
    # Prints out grid for debugging
    def print_grid(self):
        for i in range(self.height):
            for j in range(self.width):
                Cell = self.grid[i][j]
                # print(f"({Cell.x}, {Cell.y})\t", end="")
                # print(f"{Cell.distance}\t", end="")
            print()

# Path finding algorithm using Manhattan Distance
def path_finding(grid, gyro):
    curr_cell = grid.start                              # Starting cell
    End_goal = grid.goal                                # Goal cell
    min_distance = curr_cell.distance                   # Stores the minimum manhattan value
    curr_cell.direction = 'N'                           # Starting orientation of the Robot
    print(curr_cell.direction, curr_cell.distance)

    while curr_cell != End_goal:
        neighbor_cells = grid.find_neighboring_cells(curr_cell) # Find adjecent cells
        for neighbor in neighbor_cells:
            if neighbor.visited or neighbor.is_obstacle:  # Ensures that the node has not been visited, if it has been visited, it will be skipped
                continue
            if neighbor.distance < min_distance:          # Checks if new shortest distance has been found in neighboring cell and updates it
                min_distance = neighbor.distance          
                winner_cell = neighbor

        # If-else-if-else statements that determines the movement of the robot based on its current and next orientation
        if curr_cell.direction == 'N' and winner_cell.direction == 'N': 
            move_forward(gyro)
        elif curr_cell.direction == 'N' and winner_cell.direction == 'E': 
            turn_right(gyro)
            move_forward(gyro)
        elif curr_cell.direction == 'N' and winner_cell.direction == 'W': 
            turn_left(gyro)
            move_forward(gyro)
        elif curr_cell.direction == 'E' and winner_cell.direction == 'E': 
            move_forward(gyro)                                     
        elif curr_cell.direction == 'E' and winner_cell.direction == 'N': 
            turn_left(gyro)
            move_forward(gyro)
        elif curr_cell.direction == 'E' and winner_cell.direction == 'S': 
            turn_right(gyro)
            move_forward(gyro)
        elif curr_cell.direction == 'S' and winner_cell.direction == 'E': 
            turn_left(gyro)
            move_forward(gyro)
        elif curr_cell.direction == 'S' and winner_cell.direction == 'S': 
            move_forward(gyro)
        elif curr_cell.direction == 'S' and winner_cell.direction == 'W': 
            turn_right(gyro)
            move_forward(gyro)
        elif curr_cell.direction == 'W' and winner_cell.direction == 'N': 
            turn_right(gyro)
            move_forward(gyro)
        elif curr_cell.direction == 'W' and winner_cell.direction == 'S': 
            turn_left(gyro)
            move_forward()
        elif curr_cell.direction == 'W' and winner_cell.direction == 'W': 
            move_forward(gyro)
        else:
            print("Error: Invalid Orientation")
            continue

        # Sets the winner cell as visited and as the current cell
        winner_cell.set_visited()
        curr_cell = winner_cell   
        

# Moves forward a set distance of roughly 1-1.5 tiles depending on calibration
def move_forward(gyro):
    global angle
    time = 4800
    left_motor.run_time(150, time, Stop.HOLD, False)
    right_motor.run_time(150, time, Stop.HOLD, True)
    gyro.reset_angle(0)
    
# Turns the robot 90 degrees couterclockwise (left)
def turn_left(gyro):
    global angle
    angle = gyro.angle()
    while not (angle <= -90 + ERROR_L):
        left_motor.run(-100)
        right_motor.run(100)
        angle = gyro.angle()
    left_motor.stop()
    right_motor.stop()

# Turns the robot 90 degrees clockwise (right)
def turn_right(gyro):
    global angle
    angle = gyro.angle()
    while not (angle >= 90 + ERROR_R):
        left_motor.run(100)
        right_motor.run(-100)
        angle = gyro.angle()
    left_motor.stop()
    right_motor.stop()

# Locations of obstacles, start position, and goal position
obstacles = [
    [1.219, 0.305], [1.219, 0.61], [1.219, 0.915], [1.219, 1.219],
    [1.219, 1.524], [2.134, 1.219], [2.134, 1.524], [2.134, 1.83],
    [2.134, 2.134], [2.134, 2.44], [2.134, 2.743], [2.134, 3.05],
    [3.05, 0.915],
    [3.049, 1.219], [3.049, 1.524], [3.049, 1.83], [3.049, 2.134], [3.354, 0.915], [3.659, 0.915],
    [3.659, 1.219], [3.964, 0.915], [3.964, 1.219], [-1, -1], [-1, -1]
]
start = [0.61, 0.61]
goal = [3.964, 2.134]

# Instantiate motors and gyro
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
gyro = GyroSensor(Port.S2)

# Instantiate configuration space
c_space = Grid(GRID_WIDTH, GRID_HEIGHT)
c_space.set_grid_xy()               

# Populate configuration space with start, goal, and obstacles
c_space.set_obstacles(obstacles)
c_space.set_start(start)
c_space.set_goal(goal)

# Calculate Manhattan Distance for each cell
c_space.set_manhattan_values()

# Print configuration space w/ updated Manhattan values for debugging
# c_space.print_grid()

# Reset gyro angle and begin path finding
gyro.reset_angle(0)
path_finding(c_space, gyro)
