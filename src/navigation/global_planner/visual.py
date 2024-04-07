import matplotlib.pyplot as plt

def read_points(file_path):
    """Read points from a file. Each line contains x, y coordinates."""
    with open(file_path, 'r') as file:
        return [list(map(float, line.split(','))) for line in file.readlines()]

def read_obstacles(file_path):
    """Read obstacles from a file. Each line contains start and end points (x1, y1, x2, y2) of a line segment, separated by commas or spaces."""
    obstacles = []
    with open(file_path, 'r') as file:
        for line in file:
            # Split the line by commas or spaces and filter out empty strings
            parts = [p for p in line.replace(',', ' ').split() if p]
            if len(parts) == 4:  # Expecting 4 values per line
                obstacles.append(list(map(float, parts)))
    return obstacles

# Load data
tree_data = read_points("tree_structure.txt")
path_data = read_points("final_path.txt")
obstacles = read_obstacles("../../../../amrl_maps/GDC1/GDC1.vectormap.txt")
# obstacles = read_obstacles("map_data.txt")

# Plot obstacles
for obstacle in obstacles:
    plt.plot([obstacle[0], obstacle[2]], [obstacle[1], obstacle[3]], 'k-', linewidth=0.5, label='Obstacles' if obstacles.index(obstacle) == 0 else "")

# Plot tree
for line in tree_data:
    plt.plot([line[0], line[2]], [line[1], line[3]], 'r-', alpha=0.5, label='Tree' if tree_data.index(line) == 0 else "")

# Plot path
path_x, path_y = zip(*path_data)
plt.plot(path_x, path_y, 'b-', linewidth=1, label='Path')

# Assuming the first point in the path data is the start and the last is the goal
start_x, start_y = path_data[-1]
goal_x, goal_y = path_data[0]

# Plot start and goal points
plt.plot(start_x, start_y, 'go', markersize=5, label='Start')  # Start in green
plt.plot(goal_x, goal_y, 'ro', markersize=5, label='Goal')  # Goal in red

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Path Planning under Constraints using RRT Algorithm')
plt.legend()
plt.axis('equal')
plt.show()
