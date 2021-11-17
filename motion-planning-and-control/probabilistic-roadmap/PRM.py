import numpy as np
import csv
from a_star import a_star_search

def node_collision(x,y, obstacles):
    """ Checks a point for collisions with each obstacle
    Output:
    - True means this point collides with an obstacle
    - False means point is free of collisions
    """
    obs_x = obstacles[:,0]
    obs_y = obstacles[:,1]
    obs_rad = obstacles[:,2]/2
    len_obs = obstacles.shape[0]
    
    for i in range(len_obs):
        dist = calculate_distance(x,y,obs_x[i],obs_y[i])
        if dist > 1.1*obs_rad[i]:
            col = False
        else:
            return True
    return False

def edge_collision(x1,y1,x2,y2,obstacles):
    """ Output: 
    - True means there is a collision for this edge
    - False means there are no collisions for this edge
    """
    # Check endpoints of line
    col1 = node_collision(x1,y1,obstacles)
    col2 = node_collision(x2,y2,obstacles)

    if col1 or col2 == True:
        return True

    # Check collision between line and obstacles
    dist = calculate_distance(x1,y1,x2,y2)
    
    for obstacle in obstacles:
        if dist != 0:
            dot = (((obstacle[0]-x1)*(x2-x1)) + ((obstacle[1]-y1)*(y2-y1))) / (dist**2)

            # Get closest point on perpendicular line
            closest_x = x1 + dot*(x2-x1)
            closest_y = y1 + dot*(y2-y1)

            # Check if this point collides
            if node_collision(closest_x, closest_y, obstacles) == True:
                return True

    return False

def find_edges(node1):
    """ Checks for nodes with a collision free edge from input node
    """
    num_nodes = len(nodes)
    x1 = nodes[node1-1][1]
    y1 = nodes[node1-1][2]

    for i in range(num_nodes):
        x2 = nodes[i][1]
        y2 = nodes[i][2]
        if edge_collision(x1, y1, x2, y2, obstacles) == False:
            dist = calculate_distance(x1, y1, x2, y2)
            if (i+1 != node1) & (dist < 0.5):
                edge_info = np.array([node1, i+1, dist])
                edges.append(edge_info)
    return

def calculate_distance(x1,y1,x2,y2):
    dist = np.sqrt((x1-x2)**2 + (y1-y2)**2)
    return dist

# PRM Search
start = [-0.5,-0.5]
goal = [0.5,0.5]

# Get obstacles from csv file
obstacles = np.loadtxt(r'C:\Users\Summi\Documents\modern-robotics\motion-planning-and-control\probabilistic-roadmap\results\obstacles.csv',delimiter=',')

# Empty lists for nodes and edges
nodes = []
edges = []
node1 = np.array([1, -0.5, -0.5, 1.4142])
node2 = np.array([2, 0.5, 0.5, 0])
nodes.append(node1)
nodes.append(node2)

# Choose random samples
N = 100
node_id = 3
for i in range(N):
    x = np.random.uniform(start[0], goal[0])
    y = np.random.uniform(start[1], goal[1])
    # Check for collisions
    col = node_collision(x,y, obstacles)
    if col == False:
        dist = calculate_distance(x,y,goal[0], goal[1])
        node_values = np.array([node_id, x, y, dist])
        nodes.append(node_values)
        node_id += 1

# Create writers for csv files
f1 = open(r'C:\Users\Summi\Documents\modern-robotics\motion-planning-and-control\probabilistic-roadmap\results\nodes.csv', 'w', newline='')
writer1 = csv.writer(f1)
f2 = open(r'C:\Users\Summi\Documents\modern-robotics\motion-planning-and-control\probabilistic-roadmap\results\edges.csv', 'w', newline='')
writer2 = csv.writer(f2)

# Save nodes in csv file
for node in nodes:
    writer1.writerow(node)
    find_edges(int(node[0]))    
# Close the csv file
f1.close()

# Save edges in csv file
for edge in edges:
    writer2.writerow(edge)
# Close the csv file
f2.close()

# Use A* to find path after finding nodes and edges
route = a_star_search(np.array(nodes), np.array(edges))
print("Route:", route)